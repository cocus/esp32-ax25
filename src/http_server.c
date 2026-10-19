/*
 * HTTP server.
 *
 *   GET  /...      files from the "www" SPIFFS partition (the data/ folder of the project),
 *                  "/" serves index.html
 *   POST /update   either a firmware image or an image of the www partition:
 *                    pio run -e esp32dev_ota -t upload      (firmware, reboots afterwards)
 *                    pio run -e esp32dev_ota -t uploadfs    (web files, no reboot)
 *                  The kind is worked out from the data: a filesystem image is exactly the
 *                  size of the www partition, firmware starts with the ESP image magic byte.
 *                  Add ?target=app or ?target=www to say it explicitly.
 *
 * The /api/... handlers live in web_api.c.
 */
#include <stdio.h>
#include <string.h>
#include <sys/param.h>

#include <esp_http_server.h>
#include <esp_log.h>
#include <esp_ota_ops.h>
#include <esp_partition.h>
#include <esp_spiffs.h>
#include <esp_system.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

static const char TAG[] = "http";

#define WWW_LABEL "www"
#define WWW_MOUNT "/www"
#define ESP_IMAGE_MAGIC 0xE9

static bool s_www_mounted;
static char s_buf[2048]; /* the server handles one request at a time */

/* ------------------------------------------------------------------ www partition */

static void www_mount(void)
{
    esp_vfs_spiffs_conf_t conf = {
        .base_path = WWW_MOUNT,
        .partition_label = WWW_LABEL,
        .max_files = 4,
        .format_if_mount_failed = false,
    };
    esp_err_t err = esp_vfs_spiffs_register(&conf);
    s_www_mounted = err == ESP_OK;
    if (!s_www_mounted)
    {
        ESP_LOGW(TAG, "no web files (%s), upload them with: pio run -e esp32dev_ota -t uploadfs", esp_err_to_name(err));
        return;
    }
    size_t total = 0, used = 0;
    esp_spiffs_info(WWW_LABEL, &total, &used);
    ESP_LOGI(TAG, "web files mounted at %s, %u of %u bytes used", WWW_MOUNT, (unsigned)used, (unsigned)total);
}

static void www_unmount(void)
{
    if (s_www_mounted)
        esp_vfs_spiffs_unregister(WWW_LABEL);
    s_www_mounted = false;
}

/* ------------------------------------------------------------------ static files */

static const char *content_type(const char *path)
{
    static const struct
    {
        const char *ext, *type;
    } types[] = {
        {".html", "text/html"},
        {".js", "text/javascript"},
        {".css", "text/css"},
        {".json", "application/json"},
        {".svg", "image/svg+xml"},
        {".png", "image/png"},
        {".ico", "image/x-icon"},
        {".txt", "text/plain"},
    };
    const char *dot = strrchr(path, '.');
    for (size_t i = 0; dot && i < sizeof(types) / sizeof(types[0]); i++)
        if (!strcasecmp(dot, types[i].ext))
            return types[i].type;
    return "application/octet-stream";
}

static esp_err_t static_get(httpd_req_t *req)
{
    if (!s_www_mounted)
    {
        httpd_resp_set_status(req, "503 Service Unavailable");
        return httpd_resp_sendstr(req, "The web files aren't on the board yet. "
                                       "Run: pio run -e esp32dev_ota -t uploadfs\n");
    }

    /* "/www" + the URI without its query string, and index.html for directories */
    char path[64];
    size_t uri_len = strcspn(req->uri, "?#");
    if (uri_len + sizeof(WWW_MOUNT "index.html") > sizeof(path) || memmem(req->uri, uri_len, "..", 2))
    {
        httpd_resp_send_err(req, HTTPD_404_NOT_FOUND, NULL);
        return ESP_FAIL;
    }
    snprintf(path, sizeof(path), WWW_MOUNT "%.*s%s", (int)uri_len, req->uri,
             req->uri[uri_len - 1] == '/' ? "index.html" : "");

    FILE *f = fopen(path, "rb");
    if (!f)
    {
        httpd_resp_send_err(req, HTTPD_404_NOT_FOUND, NULL);
        return ESP_FAIL;
    }

    httpd_resp_set_type(req, content_type(path));
    httpd_resp_set_hdr(req, "Cache-Control", "no-cache"); /* files change with every uploadfs */
    size_t n;
    esp_err_t err = ESP_OK;
    while (err == ESP_OK && (n = fread(s_buf, 1, sizeof(s_buf), f)) > 0)
        err = httpd_resp_send_chunk(req, s_buf, n);
    fclose(f);
    if (err == ESP_OK)
        err = httpd_resp_send_chunk(req, NULL, 0);
    return err;
}

/* ------------------------------------------------------------------ updates */

/* Read up to len bytes of the body, retrying on timeouts. Returns bytes read, or -1. */
static int recv_body(httpd_req_t *req, char *buf, int len)
{
    for (;;)
    {
        int n = httpd_req_recv(req, buf, len);
        if (n == HTTPD_SOCK_ERR_TIMEOUT)
            continue;
        return n > 0 ? n : -1;
    }
}

static esp_err_t update_app(httpd_req_t *req, int first_len)
{
    const esp_partition_t *part = esp_ota_get_next_update_partition(NULL);
    esp_ota_handle_t ota;
    if (!part || esp_ota_begin(part, OTA_SIZE_UNKNOWN, &ota) != ESP_OK)
    {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Can't start the update");
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "firmware update, %d bytes into '%s'", req->content_len, part->label);

    int done = 0, n = first_len;
    while (n > 0)
    {
        if (esp_ota_write(ota, s_buf, n) != ESP_OK)
        {
            esp_ota_abort(ota);
            httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Flash write failed");
            return ESP_FAIL;
        }
        done += n;
        if (done >= (int)req->content_len)
            break;
        n = recv_body(req, s_buf, MIN((int)req->content_len - done, (int)sizeof(s_buf)));
    }
    if (n < 0)
    {
        esp_ota_abort(ota);
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Upload interrupted");
        return ESP_FAIL;
    }

    if (esp_ota_end(ota) != ESP_OK || esp_ota_set_boot_partition(part) != ESP_OK)
    {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "The image didn't validate");
        return ESP_FAIL;
    }
    httpd_resp_sendstr(req, "Firmware update complete, rebooting now!\n");
    vTaskDelay(pdMS_TO_TICKS(500));
    esp_restart();
    return ESP_OK;
}

static esp_err_t update_www(httpd_req_t *req, const esp_partition_t *part, int first_len)
{
    if (req->content_len > part->size)
    {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Image is bigger than the www partition");
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "web files update, %d bytes into '%s'", req->content_len, part->label);

    www_unmount();
    esp_err_t err = esp_partition_erase_range(part, 0, part->size);

    int done = 0, n = first_len;
    while (err == ESP_OK && n > 0)
    {
        err = esp_partition_write(part, done, s_buf, n);
        done += n;
        if (done >= (int)req->content_len)
            break;
        n = recv_body(req, s_buf, MIN((int)req->content_len - done, (int)sizeof(s_buf)));
    }
    www_mount();

    if (err != ESP_OK || n < 0)
    {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR,
                            err != ESP_OK ? "Flash write failed" : "Upload interrupted");
        return ESP_FAIL;
    }
    snprintf(s_buf, sizeof(s_buf), "Web files updated (%d bytes)%s\n", done,
             s_www_mounted ? "" : ", but the image doesn't mount as SPIFFS");
    return httpd_resp_sendstr(req, s_buf);
}

static esp_err_t update_post(httpd_req_t *req)
{
    char query[32], target[8] = "";
    if (httpd_req_get_url_query_str(req, query, sizeof(query)) == ESP_OK)
        httpd_query_key_value(query, "target", target, sizeof(target));

    int first = req->content_len > 0 ? recv_body(req, s_buf, MIN((int)req->content_len, (int)sizeof(s_buf))) : -1;
    if (first <= 0)
    {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Empty upload");
        return ESP_FAIL;
    }

    const esp_partition_t *www =
        esp_partition_find_first(ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_DATA_SPIFFS, WWW_LABEL);
    bool fs_sized = www && req->content_len == www->size;
    bool app_magic = (uint8_t)s_buf[0] == ESP_IMAGE_MAGIC;

    if (!strcmp(target, "www") || (!target[0] && fs_sized))
    {
        if (!www)
        {
            httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "No www partition");
            return ESP_FAIL;
        }
        return update_www(req, www, first);
    }
    if (!strcmp(target, "app") || (!target[0] && app_magic))
        return update_app(req, first);

    httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST,
                        "Neither a firmware image nor an image the size of the www partition");
    return ESP_FAIL;
}

/* ------------------------------------------------------------------ */

esp_err_t http_server_init(void)
{
    static httpd_handle_t server;
    extern void web_api_register(httpd_handle_t server);

    www_mount();

    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.uri_match_fn = httpd_uri_match_wildcard;
    if (httpd_start(&server, &config) != ESP_OK)
        return ESP_FAIL;

    /* Handlers are tried in the order they're registered, so the catch-all file handler goes last. */
    static const httpd_uri_t update = {.uri = "/update", .method = HTTP_POST, .handler = update_post};
    static const httpd_uri_t files = {.uri = "/*", .method = HTTP_GET, .handler = static_get};
    httpd_register_uri_handler(server, &update);
    web_api_register(server);
    httpd_register_uri_handler(server, &files);
    return ESP_OK;
}
