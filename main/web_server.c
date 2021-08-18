#include "esp_http_server.h"
#include "esp_log.h"
#include "web_server.h"
#include "messages.h"
#include "queues.h"
#include "arbitration_identifiers.h"

const char *index_html = R"EOF(
<!doctype html>
<html>
  <head>
    <title>esp32-isotp-ble-bridge</title>
  </head>
  <body>
    <script type="text/javascript">
      const uint32ToBytes = (value) => {
        return [
          value & 0xFF,
          (value >> 8) & 0xFF,
          (value >> 16) & 0xFF,
          (value >> 24) & 0xFF
        ]
      }
      const sendIsoTpRequest = (socket, requestArbitrationId, replyArbitrationId, frame) => {
        const requestArbitrationIdBytes = uint32ToBytes(requestArbitrationId)
        const replyArbitrationIdBytes = uint32ToBytes(replyArbitrationId)
        const payload = [].concat(
          requestArbitrationIdBytes,
          replyArbitrationIdBytes,
          frame
        )
        socket.send(Uint8Array.from(payload))
      }
      const run = async () => {
        const socket = new WebSocket(`ws://${window.location.host}/ws`)
        await new Promise(resolve => socket.addEventListener('open', resolve))
        socket.addEventListener('message', (event) => {
          console.log(event)
        })
        socket.addEventListener('close', (event) => {
          console.log(event)
        })
        setInterval(() => {
          const requestArbitrationId = 0x7E0 // ECU
          const replyArbitrationId = 0x7E8 // ECU
          const pdu = [0x3E, 0x00] // tester present
          sendIsoTpRequest(socket, requestArbitrationId, replyArbitrationId, pdu)
        }, 1000)
      }
      run()
    </script>
  </body>
</html>
)EOF";

const char *WEB_SERVER_TAG = "web_server";

httpd_handle_t *current_websocket_handle = NULL;
int current_websocket_fd = -1;

struct async_resp_arg {
    httpd_handle_t handle;
    int fd;
    send_message_t event;
};

void ws_async_send(void *arg)
{
    struct async_resp_arg *resp_arg = arg;
    httpd_handle_t handle = resp_arg->handle;
    int fd = resp_arg->fd;
    send_message_t event = resp_arg->event;
    httpd_ws_frame_t ws_pkt;
    memset(&ws_pkt, 0, sizeof(httpd_ws_frame_t));
    // format is: RX_ID TX_ID PDU
    update_send_identifier(read_uint32_be(event.buffer));
    update_receive_identifier(read_uint32_be(event.buffer + 4));
    ws_pkt.payload = (uint8_t*)event.buffer + 8;
    ws_pkt.len = event.msg_length - 8;
    ws_pkt.type = HTTPD_WS_TYPE_BINARY;
    httpd_ws_send_frame_async(handle, fd, &ws_pkt);
    free(resp_arg);
}

esp_err_t websocket_handler(httpd_req_t *req)
{
    if (req->method == HTTP_GET) {
        ESP_LOGI(WEB_SERVER_TAG, "Handshake done, the new connection was opened");
        return ESP_OK;
    }
    current_websocket_handle = req->handle;
    current_websocket_fd = httpd_req_to_sockfd(req);
    httpd_ws_frame_t ws_pkt;
    size_t max_len = 512 + 4; // largest PDU + 4 bytes for arbitration ID TODO: PDU max length should be 0x1000 (0xFFF + serviceId)
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
        // format is: RX_ID TX_ID PDU
        update_send_identifier(read_uint32_be(ws_pkt.payload));
        update_receive_identifier(read_uint32_be(ws_pkt.payload + 4));
        send_message_t msg;
        msg.buffer = calloc(1, ws_pkt.len - 8);
        memcpy(msg.buffer, ws_pkt.payload + 8, ws_pkt.len - 8);
        msg.msg_length = ws_pkt.len - 8;
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

void websocket_send(const void* src, size_t size) {
    send_message_t msg;
    msg.buffer = malloc(size);
    msg.msg_length = size;
    memcpy(msg.buffer, src, size);
    xQueueSend(websocket_send_queue, &msg, 50 / portTICK_PERIOD_MS);
}

void websocket_send_task(void *pvParameters)
{
    send_message_t event;
    while (1) {
        if (xQueueReceive(websocket_send_queue, (void*)&event, (TickType_t)portMAX_DELAY)) {
            ESP_LOGI(WEB_SERVER_TAG, "Got WebSocket message to send with length %08X", event.msg_length);
            if (current_websocket_handle == NULL) {
                ESP_LOGI(WEB_SERVER_TAG, "no connected websocket; skipping");
                continue;
            }
            struct async_resp_arg *resp_arg = malloc(sizeof(struct async_resp_arg));
            resp_arg->handle = current_websocket_handle;
            resp_arg->fd = current_websocket_fd;
            ESP_ERROR_CHECK(httpd_queue_work(current_websocket_handle, ws_async_send, resp_arg));
        }
    }
    vTaskDelete(NULL);
}

void web_server_setup()
{
    httpd_handle_t server = NULL;
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    ESP_LOGI(WEB_SERVER_TAG, "Starting server on port: %d", config.server_port);
    if (httpd_start(&server, &config) == ESP_OK) {
        // Registering the ws handler
        ESP_LOGI(WEB_SERVER_TAG, "Registering URI handlers");
        httpd_register_uri_handler(server, &index_uri);
        httpd_register_uri_handler(server, &websocket_uri);
        return;
    }
    ESP_LOGI(WEB_SERVER_TAG, "Error starting server!");
}
