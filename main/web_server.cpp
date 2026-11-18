#include "web_server.h"
#include "servo_controller.h"
#include "command_processor.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

static const char *TAG = "WEB_SERVER";
static httpd_handle_t server = NULL;

static const char INDEX_HTML[] = R"rawliteral(
<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>ESP32-S3 MG996R Servo API</title>
    <style>
        body { background: #020617; color: #f8fafc; font-family: -apple-system, sans-serif; display: flex; justify-content: center; align-items: center; min-height: 100vh; margin: 0; padding: 16px; }
        .card { background: #0f172a; border: 1px solid #334155; border-radius: 16px; padding: 24px; max-width: 400px; width: 100%; text-align: center; }
        h1 { font-size: 1.2rem; color: #38bdf8; margin-bottom: 8px; }
        p { font-size: 0.85rem; color: #94a3b8; line-height: 1.4; margin-bottom: 16px; }
        .badge { display: inline-block; padding: 4px 10px; border-radius: 9999px; background: #10b98122; color: #34d399; font-size: 0.75rem; font-weight: 600; }
    </style>
</head>
<body>
    <div class="card">
        <h1>MG996R Servo Controller</h1>
        <p>REST API &amp; BLE Nordic UART services are online on GPIO 1.</p>
        <span class="badge">Online</span>
    </div>
</body>
</html>
)rawliteral";

static esp_err_t root_get_handler(httpd_req_t *req) {
    httpd_resp_set_type(req, "text/html");
    httpd_resp_send(req, INDEX_HTML, HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

static esp_err_t set_angle_handler(httpd_req_t *req) {
    char query[64] = {0};
    if (httpd_req_get_url_query_str(req, query, sizeof(query)) == ESP_OK) {
        char param[16] = {0};
        if (httpd_query_key_value(query, "angle", param, sizeof(param)) == ESP_OK ||
            httpd_query_key_value(query, "val", param, sizeof(param)) == ESP_OK) {
            int requested_angle = atoi(param);
            servo_set_angle(requested_angle);
            command_notify_status();

            char resp[32];
            snprintf(resp, sizeof(resp), "OK %d", servo_get_angle());
            httpd_resp_set_type(req, "text/plain");
            httpd_resp_send(req, resp, HTTPD_RESP_USE_STRLEN);
            return ESP_OK;
        }
    }
    httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Missing 'angle' query param");
    return ESP_FAIL;
}

static esp_err_t get_status_handler(httpd_req_t *req) {
    char json_resp[128];
    snprintf(json_resp, sizeof(json_resp),
             "{\"angle\":%d,\"offset\":%d,\"enabled\":%d,\"autoidle\":%d}",
             servo_get_angle(), servo_get_offset(),
             servo_is_enabled() ? 1 : 0,
             servo_get_auto_idle() ? 1 : 0);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, json_resp, HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

static esp_err_t home_handler(httpd_req_t *req) {
    servo_move_smooth(90, 450);
    command_notify_status();
    httpd_resp_set_type(req, "text/plain");
    httpd_resp_send(req, "OK 90", HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

static esp_err_t detach_handler(httpd_req_t *req) {
    servo_detach();
    command_notify_status();
    httpd_resp_set_type(req, "text/plain");
    httpd_resp_send(req, "DETACHED", HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

void web_server_init(void) {
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.server_port = 80;
    config.max_uri_handlers = 8;
    config.lru_purge_enable = true;

    if (httpd_start(&server, &config) == ESP_OK) {
        httpd_uri_t uri_root = { .uri = "/", .method = HTTP_GET, .handler = root_get_handler, .user_ctx = NULL };
        httpd_register_uri_handler(server, &uri_root);

        httpd_uri_t uri_set = { .uri = "/set", .method = HTTP_GET, .handler = set_angle_handler, .user_ctx = NULL };
        httpd_register_uri_handler(server, &uri_set);

        httpd_uri_t uri_get = { .uri = "/get", .method = HTTP_GET, .handler = get_status_handler, .user_ctx = NULL };
        httpd_register_uri_handler(server, &uri_get);

        httpd_uri_t uri_home = { .uri = "/home", .method = HTTP_GET, .handler = home_handler, .user_ctx = NULL };
        httpd_register_uri_handler(server, &uri_home);

        httpd_uri_t uri_detach = { .uri = "/detach", .method = HTTP_GET, .handler = detach_handler, .user_ctx = NULL };
        httpd_register_uri_handler(server, &uri_detach);

        ESP_LOGI(TAG, "Web Server active on port 80 for single MG996R servo");
    } else {
        ESP_LOGE(TAG, "Failed to start HTTP server!");
    }
}