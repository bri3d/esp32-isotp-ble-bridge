#include "esp_http_server.h"
#include "esp_log.h"
#include "web_server.h"
#include "messages.h"
#include "queues.h"

const char *WEB_SERVER_TAG = "web_server";

struct async_resp_arg {
    httpd_handle_t hd;
    int fd;
};

void ws_async_send(void *arg)
{
    uint8_t response[] = {0x00, 0x00, 0x07, 0xE8, 0x7E, 0x00}; // TODO:
    struct async_resp_arg *resp_arg = arg;
    httpd_handle_t hd = resp_arg->hd;
    int fd = resp_arg->fd;
    httpd_ws_frame_t ws_pkt;
    memset(&ws_pkt, 0, sizeof(httpd_ws_frame_t));
    ws_pkt.payload = (uint8_t*)response;
    ws_pkt.len = sizeof(response);
    ws_pkt.type = HTTPD_WS_TYPE_BINARY;
    httpd_ws_send_frame_async(hd, fd, &ws_pkt);
    free(resp_arg);
}

esp_err_t trigger_async_send(httpd_handle_t handle, httpd_req_t *req)
{
    struct async_resp_arg *resp_arg = malloc(sizeof(struct async_resp_arg));
    resp_arg->hd = req->handle;
    resp_arg->fd = httpd_req_to_sockfd(req);
    return httpd_queue_work(handle, ws_async_send, resp_arg);
}

esp_err_t websocket_handler(httpd_req_t *req)
{
    if (req->method == HTTP_GET) {
        ESP_LOGI(WEB_SERVER_TAG, "Handshake done, the new connection was opened");
        return ESP_OK;
    }
    httpd_ws_frame_t ws_pkt;
    size_t max_len = 512 + 4; // largest PDU + 4 bytes for arbitration ID TODO: PDU max length should be 0x1000
    uint8_t *buf = calloc(1, max_len);
    memset(&ws_pkt, 0, sizeof(httpd_ws_frame_t));
    ws_pkt.type = HTTPD_WS_TYPE_BINARY;
    ws_pkt.payload = buf;
    esp_err_t ret = httpd_ws_recv_frame(req, &ws_pkt, max_len);
    if (ret != ESP_OK) {
        ESP_LOGE(WEB_SERVER_TAG, "httpd_ws_recv_frame failed to get frame with %d", ret);
        return ret;
    }
    ESP_LOGI(WEB_SERVER_TAG, "frame len is %d", ws_pkt.len);
    ESP_LOGI(WEB_SERVER_TAG, "Packet type: %d", ws_pkt.type);
    if (ws_pkt.type == HTTPD_WS_TYPE_BINARY) {
        for (size_t i = 0; i < ws_pkt.len; ++i) {
            ESP_LOGI(WEB_SERVER_TAG, "ws_pkt.payload[%04x] = %02x", i, ws_pkt.payload[i]);
        }
        ESP_LOGI(WEB_SERVER_TAG, "adding websocket payload to send_message_queue");
        send_message_t msg;
        msg.buffer = ws_pkt.payload;
        msg.msg_length = ws_pkt.len;
        xQueueSend(send_message_queue, &msg, pdMS_TO_TICKS(50));
    }
    return ESP_OK;
}

esp_err_t index_handler(httpd_req_t *req)
{
    httpd_resp_set_hdr(req, "Content-Type", "text/html");
    httpd_resp_send(req, index_html, HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

const httpd_uri_t index_uri = {
    .uri        = "/",
    .method     = HTTP_GET,
    .handler    = index_handler,
    .user_ctx   = NULL,
    .is_websocket = false
};

const httpd_uri_t websocket_uri = {
    .uri        = "/ws",
    .method     = HTTP_GET,
    .handler    = websocket_handler,
    .user_ctx   = NULL,
    .is_websocket = true
};

httpd_handle_t web_server_setup()
{
    httpd_handle_t server = NULL;
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    // Start the httpd server
    ESP_LOGI(WEB_SERVER_TAG, "Starting server on port: '%d'", config.server_port);
    if (httpd_start(&server, &config) == ESP_OK) {
        // Registering the ws handler
        ESP_LOGI(WEB_SERVER_TAG, "Registering URI handlers");
        httpd_register_uri_handler(server, &index_uri);
        httpd_register_uri_handler(server, &websocket_uri);
        return server;
    }
    ESP_LOGI(WEB_SERVER_TAG, "Error starting server!");
    return NULL;
}
