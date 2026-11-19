#pragma once

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

#define ESPNOW_CHANNEL        1
#define ESPNOW_MAGIC          0x59444152 // "YDAR" (YD Arm)
#define ESPNOW_PAYLOAD_CMD_SZ 96

typedef enum {
    ESPNOW_ROLE_SENDER   = 1, // MPU node
    ESPNOW_ROLE_RECEIVER = 2  // Arm node
} espnow_role_t;

enum espnow_msg_type_t : uint8_t {
    ESPNOW_MSG_DISCOVERY = 0x01,
    ESPNOW_MSG_PAIR_REQ  = 0x02,
    ESPNOW_MSG_PAIR_ACK  = 0x03,
    ESPNOW_MSG_HEARTBEAT = 0x04,
    ESPNOW_MSG_SERVO_SET = 0x10,
    ESPNOW_MSG_CMD_STR   = 0x11,
    ESPNOW_MSG_STATUS    = 0x20
};

#pragma pack(push, 1)
typedef struct {
    uint32_t magic;
    uint8_t  msg_type;
    uint8_t  sender_role;
    uint16_t seq;
    uint8_t  sender_mac[6];
    union {
        struct {
            uint8_t proto_version;
        } discovery;
        struct {
            uint8_t accepted;
        } pair_resp;
        struct {
            uint8_t  s1;
            uint8_t  s2;
            uint8_t  s3;
            uint16_t move_ms;
            uint8_t  flags; // bit 0: mpu_active, bit 1: immediate
        } servo_set;
        struct {
            char cmd[ESPNOW_PAYLOAD_CMD_SZ];
        } cmd_str;
        struct {
            uint8_t s1, s2, s3;
            int8_t  o1, o2, o3;
            uint8_t en_mask;
            uint8_t flags;
            uint8_t active_slot;
        } status;
        struct {
            uint32_t uptime_ms;
        } heartbeat;
    } payload;
} espnow_packet_t;
#pragma pack(pop)

typedef void (*espnow_cmd_handler_t)(const char *cmd_str);

void espnow_sync_init(espnow_role_t role);
bool espnow_sync_is_paired(void);
void espnow_sync_get_peer_mac(uint8_t mac[6]);
int8_t espnow_sync_get_last_rssi(void);
espnow_role_t espnow_sync_get_role(void);

// Transmit servo angles wirelessly
bool espnow_sync_send_servos(uint8_t s1, uint8_t s2, uint8_t s3, uint16_t move_ms, bool mpu_active);

// Transmit text command string (e.g. "home", "detach", "action_run 2", "s1 90")
bool espnow_sync_send_cmd(const char *cmd_str);

// Transmit status from Arm Receiver to MPU Sender
bool espnow_sync_send_status(uint8_t s1, uint8_t s2, uint8_t s3,
                             int8_t o1, int8_t o2, int8_t o3,
                             uint8_t en_mask, uint8_t flags, uint8_t active_slot);

void espnow_sync_register_cmd_handler(espnow_cmd_handler_t handler);
bool espnow_sync_format_line(char *buf, size_t max_len);

#ifdef __cplusplus
}
#endif