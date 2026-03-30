/**
 * ws_server.c — WebSocket server
 *
 * Serves reconstructed grayscale frames to connected Flutter clients.
 * Push-based: call ws_broadcast() whenever a new frame is ready.
 *
 * Frame format over WebSocket (binary):
 *   Byte 0:       position (0=left, 1=center, 2=right)
 *   Bytes 1–4:    uint32 frame sequence number
 *   Bytes 5–end:  raw grayscale pixels, FRAME_W × FRAME_H, row-major
 */

#include "ws_server.h"
#include "config.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include <string.h>

static const char *TAG = "WS";

static httpd_handle_t server      = NULL;
static int            client_fds[WS_MAX_CLIENTS];
static int            client_count = 0;

/* ------------------------------------------------------------------ */

static void add_client(int fd)
{
    if (client_count >= WS_MAX_CLIENTS) return;
    client_fds[client_count++] = fd;
    ESP_LOGI(TAG, "client connected fd=%d (%d total)", fd, client_count);
}

static void remove_client(int fd)
{
    for (int i = 0; i < client_count; i++) {
        if (client_fds[i] == fd) {
            client_fds[i] = client_fds[--client_count];
            ESP_LOGI(TAG, "client disconnected fd=%d (%d total)", fd, client_count);
            return;
        }
    }
}

/* ------------------------------------------------------------------ */

static esp_err_t ws_handler(httpd_req_t *req)
{
    if (req->method == HTTP_GET) {
        /* Handshake */
        add_client(httpd_req_to_sockfd(req));
        return ESP_OK;
    }

    httpd_ws_frame_t frame = { .type = HTTPD_WS_TYPE_BINARY };

    /* Get frame length */
    esp_err_t ret = httpd_ws_recv_frame(req, &frame, 0);
    if (ret != ESP_OK) return ret;

    /* We don't expect data from client currently — just drain it */
    if (frame.len > 0) {
        uint8_t *buf = malloc(frame.len);
        if (!buf) return ESP_ERR_NO_MEM;
        frame.payload = buf;
        ret = httpd_ws_recv_frame(req, &frame, frame.len);
        free(buf);
    }

    return ret;
}

static const httpd_uri_t ws_uri = {
    .uri         = WS_PATH,
    .method      = HTTP_GET,
    .handler     = ws_handler,
    .is_websocket = true,
};

/* ------------------------------------------------------------------ */

esp_err_t ws_server_init(void)
{
    memset(client_fds, -1, sizeof(client_fds));

    httpd_config_t cfg  = HTTPD_DEFAULT_CONFIG();
    cfg.server_port     = WS_PORT;
    cfg.ctrl_port       = WS_PORT + 1;
    cfg.max_open_sockets = WS_MAX_CLIENTS + 3;  /* +3 for internal sockets */

    esp_err_t ret = httpd_start(&server, &cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "start failed: %s", esp_err_to_name(ret));
        return ret;
    }

    httpd_register_uri_handler(server, &ws_uri);
    ESP_LOGI(TAG, "WebSocket server ready on ws://192.168.4.1:%d%s",
             WS_PORT, WS_PATH);
    return ESP_OK;
}

/* ------------------------------------------------------------------ */

esp_err_t ws_broadcast(uint8_t position, uint32_t seq,
                        const uint8_t *pixels, size_t len)
{
    if (client_count == 0) return ESP_OK;

    /* Build header: 1 byte position + 4 bytes seq */
    const size_t header_len = 5;
    uint8_t header[5];
    header[0] = position;
    header[1] = (seq >> 24) & 0xFF;
    header[2] = (seq >> 16) & 0xFF;
    header[3] = (seq >>  8) & 0xFF;
    header[4] = (seq      ) & 0xFF;

    /* Send header + pixels as two fragments or allocate one buffer.
     * esp_http_server doesn't support scatter-gather so combine them. */
    const size_t total = header_len + len;
    uint8_t *buf = malloc(total);
    if (!buf) return ESP_ERR_NO_MEM;
    memcpy(buf, header, header_len);
    memcpy(buf + header_len, pixels, len);

    httpd_ws_frame_t frame = {
        .type    = HTTPD_WS_TYPE_BINARY,
        .payload = buf,
        .len     = total,
    };

    for (int i = client_count - 1; i >= 0; i--) {
        esp_err_t ret = httpd_ws_send_frame_async(server, client_fds[i], &frame);
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "send failed fd=%d — removing client", client_fds[i]);
            remove_client(client_fds[i]);
        }
    }

    free(buf);
    return ESP_OK;
}
