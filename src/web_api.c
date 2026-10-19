/*
 * JSON and binary endpoints for the web page (the page itself is in data/, served from SPIFFS):
 *   GET  /api/scope   ?n=16..4096 -> latest raw 48 kHz ADC codes, little-endian uint16, oldest first
 *   GET  /api/status  stats and recent frames as JSON
 *   POST /api/send    body = bytes to send as one frame
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <esp_http_server.h>
#include <esp_timer.h>
#include <esp_wifi.h>

#include "afsk_modem.h"
#include "kiss_server.h"

static esp_err_t scope_get(httpd_req_t *req)
{
    static uint16_t buf[AFSK_MODEM_SCOPE_LEN]; /* the HTTP server handles one request at a time */
    char query[32], val[8];
    size_t n = 2048;
    if (httpd_req_get_url_query_str(req, query, sizeof(query)) == ESP_OK &&
        httpd_query_key_value(query, "n", val, sizeof(val)) == ESP_OK)
        n = strtoul(val, NULL, 10);
    if (n < 16)
        n = 16;
    n = afsk_modem_scope_read(buf, n);

    httpd_resp_set_type(req, "application/octet-stream");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    return httpd_resp_send(req, (const char *)buf, n * sizeof(uint16_t));
}

static void json_string(char *out, size_t size, const char *s)
{
    size_t n = 0;
    out[n++] = '"';
    for (; *s && n < size - 8; s++)
    {
        unsigned char ch = (unsigned char)*s;
        if (ch == '"' || ch == '\\')
        {
            out[n++] = '\\';
            out[n++] = ch;
        }
        else if (ch < 0x20)
            n += snprintf(out + n, size - n, "\\u%04x", ch);
        else
            out[n++] = ch;
    }
    out[n++] = '"';
    out[n] = '\0';
}

static esp_err_t status_get(httpd_req_t *req)
{
    static afsk_modem_frame_rec_t recs[AFSK_MODEM_HISTORY_LEN];
    static char line[512], text[400];

    afsk_modem_stats_t st;
    afsk_modem_get_stats(&st);
    kiss_server_stats_t ks;
    kiss_server_get_stats(&ks);
    wifi_ap_record_t ap = {0};
    esp_wifi_sta_get_ap_info(&ap);
    int64_t now = esp_timer_get_time();

    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    snprintf(line, sizeof(line),
             "{\"uptime\":%lld,\"rssi\":%d,\"rate\":%.1f,\"adc_min\":%u,\"adc_max\":%u,\"mean\":%.1f,\"rms\":%.2f,"
             "\"mark\":%.4f,\"space\":%.4f,\"flags\":%lu,\"ok\":%lu,\"bad\":%lu,\"sent\":%lu,\"tx\":%s,\"overruns\":%lu,"
             "\"dcd\":%s,\"dcd_ms\":%lu,\"kiss\":%d,"
             "\"frames\":[",
             (long long)(now / 1000000), ap.rssi, st.sample_rate, st.adc_min, st.adc_max, st.adc_mean, st.adc_rms,
             st.mark_level, st.space_level, (unsigned long)st.flags_per_sec, (unsigned long)st.frames_ok,
             (unsigned long)st.frames_bad_crc, (unsigned long)st.frames_sent, st.transmitting ? "true" : "false",
             (unsigned long)st.adc_overflows, st.dcd ? "true" : "false", (unsigned long)st.dcd_ms, ks.clients);
    httpd_resp_sendstr_chunk(req, line);

    int n = afsk_modem_get_history(recs, AFSK_MODEM_HISTORY_LEN);
    for (int i = 0; i < n; i++)
    {
        json_string(text, sizeof(text), recs[i].text);
        snprintf(line, sizeof(line), "%s{\"age\":%lld,\"ok\":%s,\"len\":%d,\"mark\":%.3f,\"space\":%.3f,\"text\":%s}",
                 i ? "," : "", (long long)((now - recs[i].time_us) / 1000000), recs[i].crc_ok ? "true" : "false",
                 recs[i].len, recs[i].mark_level, recs[i].space_level, text);
        httpd_resp_sendstr_chunk(req, line);
    }
    httpd_resp_sendstr_chunk(req, "]}");
    return httpd_resp_sendstr_chunk(req, NULL);
}

static esp_err_t send_post(httpd_req_t *req)
{
    uint8_t body[AFSK_MODEM_MAX_PAYLOAD];
    if (req->content_len < 1 || req->content_len > sizeof(body))
        return httpd_resp_sendstr(req, "need 1..328 bytes");

    int got = 0;
    while (got < (int)req->content_len)
    {
        int r = httpd_req_recv(req, (char *)body + got, req->content_len - got);
        if (r == HTTPD_SOCK_ERR_TIMEOUT)
            continue;
        if (r <= 0)
            return ESP_FAIL;
        got += r;
    }
    esp_err_t err = afsk_modem_send_raw(body, got);
    char msg[48];
    snprintf(msg, sizeof(msg), "%d bytes: %s", got, esp_err_to_name(err));
    return httpd_resp_sendstr(req, msg);
}

void web_api_register(httpd_handle_t server)
{
    static const httpd_uri_t uris[] = {
        {.uri = "/api/scope", .method = HTTP_GET, .handler = scope_get},
        {.uri = "/api/status", .method = HTTP_GET, .handler = status_get},
        {.uri = "/api/send", .method = HTTP_POST, .handler = send_post},
    };
    for (size_t i = 0; i < sizeof(uris) / sizeof(uris[0]); i++)
        httpd_register_uri_handler(server, &uris[i]);
}
