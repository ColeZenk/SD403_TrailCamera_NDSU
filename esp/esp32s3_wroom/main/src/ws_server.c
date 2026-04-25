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
#include "esp_heap_caps.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "esp_spiffs.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdio.h>
#include <string.h>
#include <sys/stat.h>

static const char *TAG = "WS";

static httpd_handle_t server = NULL;
static int client_fds[WS_MAX_CLIENTS];
static int client_count = 0;

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
                        ESP_LOGI(TAG, "client disconnected fd=%d (%d total)",
                                 fd, client_count);
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

        httpd_ws_frame_t frame = {.type = HTTPD_WS_TYPE_BINARY};

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
    .uri = WS_PATH,
    .method = HTTP_GET,
    .handler = ws_handler,
    .is_websocket = true,
};

/* ------------------------------------------------------------------ */

static esp_err_t spiffs_mount(void)
{
        esp_vfs_spiffs_conf_t cfg = {
            .base_path = "/www",
            .partition_label = "www",
            .max_files = 8,
            .format_if_mount_failed = false,
        };

        esp_err_t ret = esp_vfs_spiffs_register(&cfg);
        if (ret != ESP_OK) {
                ESP_LOGE(TAG, "SPIFFS mount failed: %s", esp_err_to_name(ret));
                return ret;
        }

        size_t total = 0, used = 0;
        esp_spiffs_info("www", &total, &used);
        ESP_LOGI(TAG, "SPIFFS: %u KB used / %u KB total",
                 (unsigned)(used / 1024), (unsigned)(total / 1024));
        return ESP_OK;
}

static const char *mime_type(const char *path)
{
        const char *dot = strrchr(path, '.');
        if (!dot) return "application/octet-stream";
        if (!strcmp(dot, ".html")) return "text/html";
        if (!strcmp(dot, ".js"))   return "application/javascript";
        if (!strcmp(dot, ".json")) return "application/json";
        if (!strcmp(dot, ".png"))  return "image/png";
        if (!strcmp(dot, ".css"))  return "text/css";
        if (!strcmp(dot, ".wasm")) return "application/wasm";
        return "application/octet-stream";
}

static esp_err_t static_handler(httpd_req_t *req)
{
        char filepath[600];
        const char *uri = req->uri;

        if (!strcmp(uri, "/")) uri = "/index.html";

        snprintf(filepath, sizeof(filepath), "/www%.590s", uri);

        struct stat st;
        if (stat(filepath, &st) != 0) {
                httpd_resp_send_404(req);
                return ESP_OK;
        }

        FILE *f = fopen(filepath, "r");
        if (!f) {
                httpd_resp_send_404(req);
                return ESP_OK;
        }

        httpd_resp_set_type(req, mime_type(filepath));

        char buf[1024];
        size_t n;
        while ((n = fread(buf, 1, sizeof(buf), f)) > 0)
                httpd_resp_send_chunk(req, buf, n);
        httpd_resp_send_chunk(req, NULL, 0);

        fclose(f);
        return ESP_OK;
}

/* ------------------------------------------------------------------ */

esp_err_t ws_server_init(void)
{
        memset(client_fds, -1, sizeof(client_fds));

        esp_err_t ret = spiffs_mount();
        if (ret != ESP_OK) return ret;

        httpd_config_t cfg = HTTPD_DEFAULT_CONFIG();
        cfg.server_port = WS_PORT;
        cfg.ctrl_port = WS_PORT + 1;
        cfg.max_open_sockets = WS_MAX_CLIENTS + 3;
        cfg.uri_match_fn = httpd_uri_match_wildcard;

        ret = httpd_start(&server, &cfg);
        if (ret != ESP_OK) {
                ESP_LOGE(TAG, "start failed: %s", esp_err_to_name(ret));
                return ret;
        }

        httpd_register_uri_handler(server, &ws_uri);

        static const httpd_uri_t static_uri = {
            .uri = "/*",
            .method = HTTP_GET,
            .handler = static_handler,
        };
        httpd_register_uri_handler(server, &static_uri);

        ESP_LOGI(TAG, "WebSocket server ready on ws://192.168.4.1:%d%s",
                 WS_PORT, WS_PATH);
        return ESP_OK;
}

/* ------------------------------------------------------------------ */

#ifdef TEST_MODE_WS_PATTERNS
void ws_test_task(void *arg)
{
        (void)arg;

        uint8_t *buf = heap_caps_malloc(FRAME_BYTES, MALLOC_CAP_SPIRAM);
        if (!buf) {
                ESP_LOGE(TAG, "PSRAM alloc failed for test frame");
                vTaskDelete(NULL);
                return;
        }

        for (int i = 0; i < FRAME_BYTES; i++)
                buf[i] = (uint8_t)((i % FRAME_W) * 255 / (FRAME_W - 1));

        uint32_t seq = 0;

        for (;;) {
                ws_broadcast(1, ++seq, buf, FRAME_BYTES);
                vTaskDelay(pdMS_TO_TICKS(1000));
        }
}
#endif

esp_err_t ws_broadcast(uint8_t position, uint32_t seq, const uint8_t *pixels,
                       size_t len)
{
        if (client_count == 0) return ESP_OK;

        /* Build header: 1 byte position + 4 bytes seq */
        const size_t header_len = 5;
        uint8_t header[5];
        header[0] = position;
        header[1] = (seq >> 24) & 0xFF;
        header[2] = (seq >> 16) & 0xFF;
        header[3] = (seq >> 8) & 0xFF;
        header[4] = (seq) & 0xFF;

        /* Send header + pixels as two fragments or allocate one buffer.
         * esp_http_server doesn't support scatter-gather so combine them. */
        const size_t total = header_len + len;
        uint8_t *buf = malloc(total);
        if (!buf) return ESP_ERR_NO_MEM;
        memcpy(buf, header, header_len);
        memcpy(buf + header_len, pixels, len);

        httpd_ws_frame_t frame = {
            .type = HTTPD_WS_TYPE_BINARY,
            .payload = buf,
            .len = total,
        };

        for (int i = client_count - 1; i >= 0; i--) {
                esp_err_t ret =
                    httpd_ws_send_frame_async(server, client_fds[i], &frame);
                if (ret != ESP_OK) {
                        ESP_LOGW(TAG, "send failed fd=%d — removing client",
                                 client_fds[i]);
                        remove_client(client_fds[i]);
                }
        }

        free(buf);
        return ESP_OK;
}
