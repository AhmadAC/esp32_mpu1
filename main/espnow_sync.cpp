#include "espnow_sync.h"
#include "ble_server.h"
#include "servo_controller.h"
#include "command_processor.h"

#include <string.h>
#include <stdio.h>
#include "esp_wifi.h"
#include "esp_now.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_idf_version.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "ESPNOW_SYNC";

static const uint8_t BCAST_MAC[6] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };

static espnow_role_t s_local_role = ESPNOW_ROLE_RECEIVER;
static uint8_t s_local_mac[6] = {0};
static uint8_t s_peer_mac[6]  = {0};
static volatile bool s_is_paired = false;
static int64_t s_last_recv_us = 0;
static int8_t  s_last_rssi = 0;
static uint16_t s_tx_seq = 0;

static espnow_cmd_handler_t s_cmd_handler = NULL;
static portMUX_TYPE s_mux = portMUX_INITIALIZER_UNLOCKED;

static void notify_link_state(void);

static bool add_peer_internal(const uint8_t *mac) {
    if (esp_now_is_peer_exist(mac)) {
        return true;
    }
    esp_now_peer_info_t peer = {};
    memcpy(peer.peer_addr, mac, 6);
    peer.channel = ESPNOW_CHANNEL;
    peer.ifidx   = WIFI_IF_STA;
    peer.encrypt = false;
    esp_err_t err = esp_now_add_peer(&peer);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_now_add_peer failed: %s", esp_err_to_name(err));
        return false;
    }
    return true;
}

static bool send_packet(const uint8_t *dest_mac, espnow_packet_t *pkt) {
    pkt->magic = ESPNOW_MAGIC;
    pkt->sender_role = (uint8_t)s_local_role;
    pkt->seq = s_tx_seq++;
    memcpy(pkt->sender_mac, s_local_mac, 6);

    esp_err_t err = esp_now_send(dest_mac, (const uint8_t *)pkt, sizeof(*pkt));
    return (err == ESP_OK);
}

static void handle_incoming_packet(const uint8_t *src_mac, const uint8_t *data, size_t len, int8_t rssi) {
    if (len < sizeof(espnow_packet_t)) return;

    const espnow_packet_t *pkt = (const espnow_packet_t *)data;
    if (pkt->magic != ESPNOW_MAGIC) return;

    // Discard packets from ourselves
    if (memcmp(pkt->sender_mac, s_local_mac, 6) == 0) return;

    // Reject packets from the same role (e.g. two senders or two receivers)
    if (pkt->sender_role == (uint8_t)s_local_role) return;

    s_last_recv_us = esp_timer_get_time();
    s_last_rssi = rssi;

    switch (pkt->msg_type) {
    case ESPNOW_MSG_DISCOVERY: {
        ESP_LOGI(TAG, "Discovery beacon received from %02X:%02X:%02X:%02X:%02X:%02X (role=%d)",
                 src_mac[0], src_mac[1], src_mac[2], src_mac[3], src_mac[4], src_mac[5], pkt->sender_role);

        add_peer_internal(src_mac);

        if (s_local_role == ESPNOW_ROLE_SENDER) {
            espnow_packet_t req = {};
            req.msg_type = ESPNOW_MSG_PAIR_REQ;
            req.payload.discovery.proto_version = 1;
            send_packet(src_mac, &req);
        } else {
            espnow_packet_t ack = {};
            ack.msg_type = ESPNOW_MSG_PAIR_ACK;
            ack.payload.pair_resp.accepted = 1;
            send_packet(src_mac, &ack);

            portENTER_CRITICAL(&s_mux);
            memcpy(s_peer_mac, src_mac, 6);
            s_is_paired = true;
            portEXIT_CRITICAL(&s_mux);
            notify_link_state();
        }
        break;
    }

    case ESPNOW_MSG_PAIR_REQ: {
        add_peer_internal(src_mac);
        portENTER_CRITICAL(&s_mux);
        memcpy(s_peer_mac, src_mac, 6);
        s_is_paired = true;
        portEXIT_CRITICAL(&s_mux);

        espnow_packet_t ack = {};
        ack.msg_type = ESPNOW_MSG_PAIR_ACK;
        ack.payload.pair_resp.accepted = 1;
        send_packet(src_mac, &ack);

        ESP_LOGI(TAG, "Auto-paired with Master %02X:%02X:%02X:%02X:%02X:%02X",
                 src_mac[0], src_mac[1], src_mac[2], src_mac[3], src_mac[4], src_mac[5]);
        notify_link_state();
        break;
    }

    case ESPNOW_MSG_PAIR_ACK: {
        add_peer_internal(src_mac);
        portENTER_CRITICAL(&s_mux);
        memcpy(s_peer_mac, src_mac, 6);
        s_is_paired = true;
        portEXIT_CRITICAL(&s_mux);

        ESP_LOGI(TAG, "Auto-pair confirmed by Receiver %02X:%02X:%02X:%02X:%02X:%02X",
                 src_mac[0], src_mac[1], src_mac[2], src_mac[3], src_mac[4], src_mac[5]);
        notify_link_state();
        break;
    }

    case ESPNOW_MSG_HEARTBEAT: {
        // Keep-alive packet refreshed s_last_recv_us
        break;
    }

    case ESPNOW_MSG_SERVO_SET: {
        if (s_local_role == ESPNOW_ROLE_RECEIVER) {
            uint8_t s1 = pkt->payload.servo_set.s1;
            uint8_t s2 = pkt->payload.servo_set.s2;
            uint8_t s3 = pkt->payload.servo_set.s3;
            uint16_t mv = pkt->payload.servo_set.move_ms;

            if (mv > 20) {
                servo_move_smooth(s1, s2, s3, mv);
            } else {
                servo_set_angle(0, s1);
                servo_set_angle(1, s2);
                servo_set_angle(2, s3);
            }
        }
        break;
    }

    case ESPNOW_MSG_CMD_STR: {
        if (s_local_role == ESPNOW_ROLE_RECEIVER) {
            char cmd_copy[ESPNOW_PAYLOAD_CMD_SZ + 1];
            strncpy(cmd_copy, pkt->payload.cmd_str.cmd, ESPNOW_PAYLOAD_CMD_SZ);
            cmd_copy[ESPNOW_PAYLOAD_CMD_SZ] = '\0';
            ESP_LOGI(TAG, "Wireless command received: '%s'", cmd_copy);

            if (s_cmd_handler != NULL) {
                s_cmd_handler(cmd_copy);
            }
        }
        break;
    }

    case ESPNOW_MSG_STATUS: {
        if (s_local_role == ESPNOW_ROLE_SENDER) {
            uint8_t s1 = pkt->payload.status.s1;
            uint8_t s2 = pkt->payload.status.s2;
            uint8_t s3 = pkt->payload.status.s3;
            int8_t  o1 = pkt->payload.status.o1;
            int8_t  o2 = pkt->payload.status.o2;
            int8_t  o3 = pkt->payload.status.o3;
            uint8_t en = pkt->payload.status.en_mask;

            char status_line[160];
            snprintf(status_line, sizeof(status_line),
                     "STATUS:s1=%d,s2=%d,s3=%d,o1=%d,o2=%d,o3=%d,en1=%d,en2=%d,en3=%d,idle=%d,run=%d,act=%d,loop=%d,bo=0,mpuc=1,mpuok=1\n",
                     s1, s2, s3, o1, o2, o3,
                     (en & 1) ? 1 : 0, (en & 2) ? 1 : 0, (en & 4) ? 1 : 0,
                     (pkt->payload.status.flags & 1) ? 1 : 0,
                     (pkt->payload.status.flags & 2) ? 1 : 0,
                     pkt->payload.status.active_slot,
                     (pkt->payload.status.flags & 4) ? 1 : 0);
            ble_server_send_notify(status_line);
        }
        break;
    }

    default:
        break;
    }
}

#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 1, 0)
static void espnow_recv_cb(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
    if (!info || !data || len <= 0) return;
    int8_t rssi = (info->rx_ctrl != NULL) ? info->rx_ctrl->rssi : 0;
    handle_incoming_packet(info->src_addr, data, (size_t)len, rssi);
}
#else
static void espnow_recv_cb(const uint8_t *mac_addr, const uint8_t *data, int len) {
    if (!mac_addr || !data || len <= 0) return;
    handle_incoming_packet(mac_addr, data, (size_t)len, 0);
}
#endif

static void espnow_send_cb(const uint8_t *mac_addr, esp_now_send_status_t status) {
    (void)mac_addr;
    (void)status;
}

static void notify_link_state(void) {
    char line[128];
    if (espnow_sync_format_line(line, sizeof(line))) {
        ble_server_send_notify(line);
    }
}

static void espnow_sync_task(void *pvParameters) {
    (void)pvParameters;

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(800));

        int64_t now_us = esp_timer_get_time();

        // 1. Check for stale peer connection timeout (3.5 seconds)
        if (s_is_paired) {
            if (now_us - s_last_recv_us > 3500000) {
                ESP_LOGW(TAG, "ESP-NOW link lost (peer silent). Re-entering auto-discovery.");
                portENTER_CRITICAL(&s_mux);
                s_is_paired = false;
                memset(s_peer_mac, 0, 6);
                portEXIT_CRITICAL(&s_mux);
                notify_link_state();
            } else {
                // Send heartbeat to maintain link
                espnow_packet_t hb = {};
                hb.msg_type = ESPNOW_MSG_HEARTBEAT;
                hb.payload.heartbeat.uptime_ms = (uint32_t)(now_us / 1000);
                send_packet(s_peer_mac, &hb);
            }
        }

        // 2. If unpaired, broadcast discovery beacon
        if (!s_is_paired) {
            espnow_packet_t disc = {};
            disc.msg_type = ESPNOW_MSG_DISCOVERY;
            disc.payload.discovery.proto_version = 1;
            send_packet(BCAST_MAC, &disc);
        }
    }
}

void espnow_sync_init(espnow_role_t role) {
    s_local_role = role;
    s_is_paired = false;
    memset(s_peer_mac, 0, 6);

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_ERROR_CHECK(esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE));
    ESP_ERROR_CHECK(esp_wifi_get_mac(WIFI_IF_STA, s_local_mac));

    ESP_ERROR_CHECK(esp_now_init());
    ESP_ERROR_CHECK(esp_now_register_recv_cb(espnow_recv_cb));
    ESP_ERROR_CHECK(esp_now_register_send_cb(espnow_send_cb));

    add_peer_internal(BCAST_MAC);

    xTaskCreatePinnedToCore(espnow_sync_task, "espnow_task", 4096, NULL, 4, NULL, 1);

    ESP_LOGI(TAG, "ESP-NOW Auto-Sync Initialized (Role=%s, MAC=%02X:%02X:%02X:%02X:%02X:%02X, CH=%d)",
             role == ESPNOW_ROLE_SENDER ? "MPU-Sender" : "Arm-Receiver",
             s_local_mac[0], s_local_mac[1], s_local_mac[2], s_local_mac[3], s_local_mac[4], s_local_mac[5],
             ESPNOW_CHANNEL);
}

bool espnow_sync_is_paired(void) {
    return s_is_paired;
}

void espnow_sync_get_peer_mac(uint8_t mac[6]) {
    portENTER_CRITICAL(&s_mux);
    memcpy(mac, s_peer_mac, 6);
    portEXIT_CRITICAL(&s_mux);
}

int8_t espnow_sync_get_last_rssi(void) {
    return s_last_rssi;
}

espnow_role_t espnow_sync_get_role(void) {
    return s_local_role;
}

bool espnow_sync_send_servos(uint8_t s1, uint8_t s2, uint8_t s3, uint16_t move_ms, bool mpu_active) {
    if (!s_is_paired) return false;

    espnow_packet_t pkt = {};
    pkt.msg_type = ESPNOW_MSG_SERVO_SET;
    pkt.payload.servo_set.s1 = s1;
    pkt.payload.servo_set.s2 = s2;
    pkt.payload.servo_set.s3 = s3;
    pkt.payload.servo_set.move_ms = move_ms;
    pkt.payload.servo_set.flags = mpu_active ? 1 : 0;

    return send_packet(s_peer_mac, &pkt);
}

bool espnow_sync_send_cmd(const char *cmd_str) {
    if (!s_is_paired || !cmd_str) return false;

    espnow_packet_t pkt = {};
    pkt.msg_type = ESPNOW_MSG_CMD_STR;
    strncpy(pkt.payload.cmd_str.cmd, cmd_str, ESPNOW_PAYLOAD_CMD_SZ - 1);
    pkt.payload.cmd_str.cmd[ESPNOW_PAYLOAD_CMD_SZ - 1] = '\0';

    return send_packet(s_peer_mac, &pkt);
}

bool espnow_sync_send_status(uint8_t s1, uint8_t s2, uint8_t s3,
                             int8_t o1, int8_t o2, int8_t o3,
                             uint8_t en_mask, uint8_t flags, uint8_t active_slot) {
    if (!s_is_paired) return false;

    espnow_packet_t pkt = {};
    pkt.msg_type = ESPNOW_MSG_STATUS;
    pkt.payload.status.s1 = s1;
    pkt.payload.status.s2 = s2;
    pkt.payload.status.s3 = s3;
    pkt.payload.status.o1 = o1;
    pkt.payload.status.o2 = o2;
    pkt.payload.status.o3 = o3;
    pkt.payload.status.en_mask = en_mask;
    pkt.payload.status.flags = flags;
    pkt.payload.status.active_slot = active_slot;

    return send_packet(s_peer_mac, &pkt);
}

void espnow_sync_register_cmd_handler(espnow_cmd_handler_t handler) {
    s_cmd_handler = handler;
}

bool espnow_sync_format_line(char *buf, size_t max_len) {
    if (!buf || max_len < 64) return false;

    char mac_str[18] = "00:00:00:00:00:00";
    if (s_is_paired) {
        snprintf(mac_str, sizeof(mac_str), "%02X:%02X:%02X:%02X:%02X:%02X",
                 s_peer_mac[0], s_peer_mac[1], s_peer_mac[2], s_peer_mac[3], s_peer_mac[4], s_peer_mac[5]);
    }

    snprintf(buf, max_len, "ESPNOW:paired=%d,peer=%s,rssi=%d,role=%s\n",
             s_is_paired ? 1 : 0, mac_str, (int)s_last_rssi,
             s_local_role == ESPNOW_ROLE_SENDER ? "SENDER" : "RECEIVER");
    return true;
}