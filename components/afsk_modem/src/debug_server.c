#include "debug_server.h"

#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>

#include "afsk_modem.h"
#include "kiss_server.h"

#include "esp_check.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/stream_buffer.h"
#include "freertos/task.h"
#include "lwip/sockets.h"

static const char TAG[] = "afsk_dbg";

#define CONSOLE_OUT_BYTES 8192
#define CMD_MAX_LEN       400

static uint16_t s_console_port, s_audio_port;

/*
 * Console output: the modem tasks, the log hook and the console task all write
 * whole lines into s_out under s_out_lock; the console task drains it to the socket.
 * Writers never wait more than a few ms, so a slow client cannot stall the modem.
 */
static StreamBufferHandle_t s_out;
static SemaphoreHandle_t s_out_lock;
static volatile bool s_client;
static volatile uint32_t s_out_dropped;

/* what gets forwarded to the console */
static volatile bool s_show_bad;
static volatile bool s_show_level;
static volatile bool s_mirror_log;
static volatile afsk_tap_t s_tap_src = AFSK_TAP_RAW_48K;
static volatile uint32_t s_tap_capture_ms; /* 0 = live stream */
static volatile bool s_tap_busy;

static vprintf_like_t s_prev_vprintf;
static SemaphoreHandle_t s_log_lock;
static char s_log_buf[256];

/* ------------------------------------------------------------------ console output */

static void out_write_locked(const char *s, size_t len)
{
    size_t sent = xStreamBufferSend(s_out, s, len, 0);
    if (sent < len)
        s_out_dropped += len - sent;
}

/* "[  123.456] <line>\r\n" as one unit */
static void out_line(const char *line)
{
    if (!s_client)
        return;
    char ts[20];
    int64_t ms = esp_timer_get_time() / 1000;
    int n = snprintf(ts, sizeof(ts), "[%7lld.%03d] ", (long long)(ms / 1000), (int)(ms % 1000));
    if (xSemaphoreTake(s_out_lock, pdMS_TO_TICKS(5)) != pdTRUE)
    {
        s_out_dropped += strlen(line);
        return;
    }
    out_write_locked(ts, n);
    out_write_locked(line, strlen(line));
    out_write_locked("\r\n", 2);
    xSemaphoreGive(s_out_lock);
}

/* Console task only: formatted reply without timestamp. */
static void reply(const char *fmt, ...)
{
    static char buf[512];
    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    if (n > (int)sizeof(buf) - 1)
        n = sizeof(buf) - 1;
    if (xSemaphoreTake(s_out_lock, pdMS_TO_TICKS(100)) == pdTRUE)
    {
        out_write_locked(buf, n);
        xSemaphoreGive(s_out_lock);
    }
}

static void on_modem_event(afsk_dbg_event_t type, const char *line, void *ctx)
{
    if ((type == AFSK_DBG_FRAME_BAD && !s_show_bad) || (type == AFSK_DBG_LEVEL && !s_show_level))
        return;
    out_line(line);
}

static int log_vprintf(const char *fmt, va_list ap)
{
    if (s_mirror_log && s_client && xSemaphoreTake(s_log_lock, pdMS_TO_TICKS(5)) == pdTRUE)
    {
        va_list copy;
        va_copy(copy, ap);
        int n = vsnprintf(s_log_buf, sizeof(s_log_buf), fmt, copy);
        va_end(copy);
        if (n > (int)sizeof(s_log_buf) - 1)
            n = sizeof(s_log_buf) - 1;
        if (n > 0 && xSemaphoreTake(s_out_lock, pdMS_TO_TICKS(5)) == pdTRUE)
        {
            out_write_locked(s_log_buf, n);
            xSemaphoreGive(s_out_lock);
        }
        xSemaphoreGive(s_log_lock);
    }
    return s_prev_vprintf(fmt, ap);
}

/* ------------------------------------------------------------------ sockets */

static int listen_on(uint16_t port)
{
    int s = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (s < 0)
        return -1;
    int one = 1;
    setsockopt(s, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));
    struct sockaddr_in addr = {
        .sin_family = AF_INET,
        .sin_port = htons(port),
        .sin_addr.s_addr = htonl(INADDR_ANY),
    };
    if (bind(s, (struct sockaddr *)&addr, sizeof(addr)) < 0 || listen(s, 1) < 0)
    {
        close(s);
        return -1;
    }
    return s;
}

static int accept_client(int ls, char *peer, size_t peer_len)
{
    struct sockaddr_in from;
    socklen_t flen = sizeof(from);
    int c = accept(ls, (struct sockaddr *)&from, &flen);
    if (c < 0)
        return -1;
    inet_ntoa_r(from.sin_addr, peer, peer_len);
    int one = 1;
    setsockopt(c, IPPROTO_TCP, TCP_NODELAY, &one, sizeof(one));
    struct timeval tv = {.tv_sec = 3};
    setsockopt(c, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv));
    return c;
}

static bool send_all(int s, const void *data, size_t len)
{
    const uint8_t *p = data;
    while (len)
    {
        int n = send(s, p, len, 0);
        if (n <= 0)
            return false;
        p += n;
        len -= n;
    }
    return true;
}

/* ------------------------------------------------------------------ commands */

static const char HELP[] =
    "commands:\r\n"
    "  stats                 counters, levels, sample rate, heap\r\n"
    "  frames [n]            last n received frames (good and bad CRC)\r\n"
    "  level on|off          1 Hz audio level / tone level / flags-per-second meter\r\n"
    "  bad on|off            show frames that failed the CRC (hex dump)\r\n"
    "  log on|off            mirror ESP_LOG output here\r\n"
    "  quiet                 level, bad and log off\r\n"
    "  send <text>           send the text as one raw frame (no AX.25)\r\n"
    "  tx <tnc2>             send an AX.25 UI frame, e.g. tx N0CALL>TEST,WIDE1-1:hello\r\n"
    "  tone mark|space|alt [ms]  key up with a test tone (default 3000 ms)\r\n"
    "  tap 48k|12k [secs]    audio port source; with secs, record that long into RAM first and\r\n"
    "                        then send (no drops on slow WiFi; ~3 s at 12k, <1 s at 48k fit)\r\n"
    "  help, quit\r\n"
    "always shown: decoded frames and TX events\r\n";

static bool parse_on_off(const char *arg, volatile bool *flag, const char *name)
{
    if (!strcasecmp(arg, "on"))
        *flag = true;
    else if (!strcasecmp(arg, "off"))
        *flag = false;
    else
    {
        reply("%s is %s (use: %s on|off)\r\n", name, *flag ? "on" : "off", name);
        return false;
    }
    reply("%s %s\r\n", name, *flag ? "on" : "off");
    return true;
}

static void cmd_stats(void)
{
    afsk_modem_stats_t st;
    afsk_modem_get_stats(&st);
    int64_t up = esp_timer_get_time() / 1000000;
    reply("uptime %llds, free heap %u (min %u)\r\n", (long long)up,
          (unsigned)heap_caps_get_free_size(MALLOC_CAP_8BIT), (unsigned)heap_caps_get_minimum_free_size(MALLOC_CAP_8BIT));
    wifi_ap_record_t ap;
    if (esp_wifi_sta_get_ap_info(&ap) == ESP_OK)
        reply("wifi: \"%s\" rssi %d dBm, channel %d\r\n", (const char *)ap.ssid, ap.rssi, ap.primary);
    reply("ADC: %.0f samples/s, range %u..%u, mean %.0f, rms %.1f counts%s\r\n", st.sample_rate, st.adc_min,
          st.adc_max, st.adc_mean, st.adc_rms, (st.adc_min == 0 || st.adc_max == 4095) ? "  CLIPPING" : "");
    reply("tones: mark %.4f space %.4f FS, flags/s %lu, RX processing %.1f%% of a core\r\n", st.mark_level,
          st.space_level, (unsigned long)st.flags_per_sec, st.rx_cpu);
    reply("frames: ok %lu, bad CRC %lu, sent %lu, transmitting %s\r\n", (unsigned long)st.frames_ok,
          (unsigned long)st.frames_bad_crc, (unsigned long)st.frames_sent, st.transmitting ? "yes" : "no");
    afsk_modem_channel_t ch;
    afsk_modem_get_channel(&ch);
    reply("channel: DCD %s (%lu ms in the last second), txdelay %d ms, persist %d, slot %d ms, tail %d flags, %s\r\n",
          st.dcd ? "ON" : "off", (unsigned long)st.dcd_ms, ch.txdelay_ms, ch.persist, ch.slot_ms, ch.txtail_flags,
          ch.full_duplex ? "full duplex" : "half duplex");
    kiss_server_stats_t ks;
    kiss_server_get_stats(&ks);
    reply("kiss: %d client(s), frames to air %lu, from air %lu, dropped %lu, bad KISS %lu\r\n", ks.clients,
          (unsigned long)ks.to_air, (unsigned long)ks.from_air, (unsigned long)ks.dropped, (unsigned long)ks.bad_kiss);
    reply("drops: ADC overruns %lu, audio tap %lu bytes, console %lu bytes\r\n", (unsigned long)st.adc_overflows,
          (unsigned long)st.tap_dropped, (unsigned long)s_out_dropped);
    reply("console: level %s, bad %s, log %s; tap source %s%s\r\n", s_show_level ? "on" : "off",
          s_show_bad ? "on" : "off", s_mirror_log ? "on" : "off", s_tap_src == AFSK_TAP_DECIMATED_12K ? "12k" : "48k",
          s_tap_busy ? " (streaming)" : "");
}

static void cmd_frames(int max)
{
    static afsk_modem_frame_rec_t recs[AFSK_MODEM_HISTORY_LEN];
    if (max <= 0 || max > AFSK_MODEM_HISTORY_LEN)
        max = AFSK_MODEM_HISTORY_LEN;
    int n = afsk_modem_get_history(recs, max);
    int64_t now = esp_timer_get_time();
    if (!n)
        reply("no frames yet\r\n");
    for (int i = 0; i < n; i++)
        reply("%6llds ago %-3s %3d bytes mark %.3f space %.3f: %s\r\n", (long long)((now - recs[i].time_us) / 1000000),
              recs[i].crc_ok ? "ok" : "BAD", recs[i].len, recs[i].mark_level, recs[i].space_level, recs[i].text);
}

/* Returns false to close the session. */
static bool handle_command(char *line)
{
    while (*line == ' ')
        line++;
    char *arg = line;
    while (*arg && *arg != ' ')
        arg++;
    if (*arg)
        *arg++ = '\0';
    while (*arg == ' ')
        arg++;

    if (!*line)
        return true;
    if (!strcasecmp(line, "help") || !strcmp(line, "?"))
        reply("%s", HELP);
    else if (!strcasecmp(line, "quit") || !strcasecmp(line, "exit"))
        return false;
    else if (!strcasecmp(line, "stats"))
        cmd_stats();
    else if (!strcasecmp(line, "frames"))
        cmd_frames(atoi(arg));
    else if (!strcasecmp(line, "level"))
        parse_on_off(arg, &s_show_level, "level");
    else if (!strcasecmp(line, "bad"))
        parse_on_off(arg, &s_show_bad, "bad");
    else if (!strcasecmp(line, "log"))
        parse_on_off(arg, &s_mirror_log, "log");
    else if (!strcasecmp(line, "quiet"))
    {
        s_show_level = s_show_bad = s_mirror_log = false;
        reply("level, bad and log off\r\n");
    }
    else if (!strcasecmp(line, "send"))
    {
        esp_err_t err = afsk_modem_send_raw((const uint8_t *)arg, (int)strlen(arg));
        reply("send: %s\r\n", err == ESP_ERR_INVALID_ARG ? "need 1..328 bytes of text" : esp_err_to_name(err));
    }
    else if (!strcasecmp(line, "tx"))
    {
        esp_err_t err = afsk_modem_send(arg);
        reply("tx: %s\r\n", err == ESP_ERR_INVALID_ARG ? "cannot parse, expected SRC>DST[,PATH]:info" : esp_err_to_name(err));
    }
    else if (!strcasecmp(line, "tone"))
    {
        char *ms_str = arg;
        while (*ms_str && *ms_str != ' ')
            ms_str++;
        if (*ms_str)
            *ms_str++ = '\0';
        uint32_t ms = *ms_str ? (uint32_t)atoi(ms_str) : 3000;
        afsk_tone_t tone;
        if (!strcasecmp(arg, "mark"))
            tone = AFSK_TONE_MARK;
        else if (!strcasecmp(arg, "space"))
            tone = AFSK_TONE_SPACE;
        else if (!strcasecmp(arg, "alt"))
            tone = AFSK_TONE_ALTERNATE;
        else
        {
            reply("usage: tone mark|space|alt [ms]\r\n");
            return true;
        }
        reply("tone: %s\r\n", esp_err_to_name(afsk_modem_send_tone(tone, ms)));
    }
    else if (!strcasecmp(line, "tap"))
    {
        char *secs = arg;
        while (*secs && *secs != ' ')
            secs++;
        if (*secs)
            *secs++ = '\0';
        if (!strcasecmp(arg, "48k"))
            s_tap_src = AFSK_TAP_RAW_48K;
        else if (!strcasecmp(arg, "12k"))
            s_tap_src = AFSK_TAP_DECIMATED_12K;
        if (*arg)
            s_tap_capture_ms = (uint32_t)(atof(secs) * 1000.0);
        if (s_tap_capture_ms)
            reply("tap: %s, buffered %.1f s; connecting to port %u records into RAM first, then sends\r\n",
                  s_tap_src == AFSK_TAP_DECIMATED_12K ? "12k" : "48k", s_tap_capture_ms / 1000.0, s_audio_port);
        else
            reply("tap: %s, live; connect to port %u to stream it\r\n",
                  s_tap_src == AFSK_TAP_DECIMATED_12K ? "12k" : "48k", s_audio_port);
    }
    else
        reply("unknown command '%s', try help\r\n", line);
    return true;
}

/* ------------------------------------------------------------------ tasks */

static void console_task(void *arg)
{
    static char cmd[CMD_MAX_LEN];
    static uint8_t tmp[512];
    char peer[16];

    int ls = listen_on(s_console_port);
    if (ls < 0)
    {
        ESP_LOGE(TAG, "console: cannot listen on port %u", s_console_port);
        vTaskDelete(NULL);
    }

    for (;;)
    {
        int c = accept_client(ls, peer, sizeof(peer));
        if (c < 0)
            continue;
        ESP_LOGI(TAG, "console client %s", peer);

        xSemaphoreTake(s_out_lock, portMAX_DELAY);
        xStreamBufferReset(s_out);
        s_out_dropped = 0;
        s_client = true;
        xSemaphoreGive(s_out_lock);

        reply("esp32 afsk modem debug console\r\n%s", HELP);
        int cmd_len = 0;
        uint32_t reported_drops = 0;
        bool open = true;

        while (open)
        {
            fd_set rfds;
            FD_ZERO(&rfds);
            FD_SET(c, &rfds);
            struct timeval tv = {.tv_sec = 0, .tv_usec = 50000};
            if (select(c + 1, &rfds, NULL, NULL, &tv) > 0)
            {
                int n = recv(c, tmp, sizeof(tmp), 0);
                if (n <= 0)
                    break;
                for (int i = 0; i < n && open; i++)
                {
                    uint8_t ch = tmp[i];
                    if (ch == '\n' || ch == '\r')
                    {
                        if (cmd_len)
                        {
                            cmd[cmd_len] = '\0';
                            open = handle_command(cmd);
                            cmd_len = 0;
                        }
                    }
                    else if (ch >= 0x20 && ch < 0x7F && cmd_len < CMD_MAX_LEN - 1)
                    {
                        cmd[cmd_len++] = (char)ch; /* drops telnet negotiation and control bytes */
                    }
                }
            }

            if (s_out_dropped != reported_drops)
            {
                reported_drops = s_out_dropped;
                reply("[console output dropped, %lu bytes so far]\r\n", (unsigned long)reported_drops);
            }

            size_t n;
            while (open && (n = xStreamBufferReceive(s_out, tmp, sizeof(tmp), 0)) > 0)
                open = send_all(c, tmp, n);
        }

        s_client = false;
        close(c);
        ESP_LOGI(TAG, "console client %s left", peer);
    }
}

static void put_le32(uint8_t *p, uint32_t v)
{
    p[0] = v;
    p[1] = v >> 8;
    p[2] = v >> 16;
    p[3] = v >> 24;
}

/* 16-bit mono WAV header */
static void wav_header(uint8_t hdr[44], uint32_t rate, uint32_t data_bytes)
{
    memcpy(hdr, "RIFF", 4);
    put_le32(hdr + 4, 36 + data_bytes);
    memcpy(hdr + 8, "WAVEfmt ", 8);
    put_le32(hdr + 16, 16);
    put_le32(hdr + 20, 1 | (1 << 16)); /* PCM, mono */
    put_le32(hdr + 24, rate);
    put_le32(hdr + 28, rate * 2);
    put_le32(hdr + 32, 2 | (16 << 16)); /* block align 2, 16 bits */
    memcpy(hdr + 36, "data", 4);
    put_le32(hdr + 40, data_bytes);
}

static void audio_status(char *line, size_t size, const char *fmt, ...)
{
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(line, size, fmt, ap);
    va_end(ap);
    out_line(line);
    ESP_LOGI(TAG, "%s", line);
}

/* Live: stream until the client goes away. Whatever the network cannot keep up with is dropped. */
static uint64_t audio_stream_live(int c, uint32_t rate, afsk_tap_t src)
{
    static int16_t samples[1024];
    uint8_t hdr[44];
    uint64_t total = 0;

    wav_header(hdr, rate, 0x7FFFFFFF - 36); /* unknown length: declare it huge */
    bool ok = send_all(c, hdr, sizeof(hdr));
    if (ok)
        afsk_modem_tap_start(src);
    while (ok)
    {
        size_t n = afsk_modem_tap_read(samples, sizeof(samples) / sizeof(samples[0]), pdMS_TO_TICKS(200));
        if (n)
        {
            ok = send_all(c, samples, n * sizeof(int16_t));
            total += n;
        }
    }
    afsk_modem_tap_stop();
    return total;
}

/* Buffered: record into RAM without touching the network, then send at whatever speed the link allows. */
static uint64_t audio_capture(int c, uint32_t rate, afsk_tap_t src, uint32_t ms, char *line, size_t line_size)
{
    size_t want = (size_t)((uint64_t)rate * ms / 1000);
    size_t largest = heap_caps_get_largest_free_block(MALLOC_CAP_8BIT);
    size_t room = largest > 24 * 1024 ? (largest - 24 * 1024) / sizeof(int16_t) : 0; /* leave heap for WiFi */
    if (want > room)
        want = room;
    int16_t *buf = want ? malloc(want * sizeof(int16_t)) : NULL;
    if (!buf)
    {
        audio_status(line, line_size, "audio tap: no memory for a capture (largest free block %u bytes)",
                     (unsigned)largest);
        return 0;
    }

    audio_status(line, line_size, "audio tap: recording %.2f s at %lu Hz into RAM (%u bytes)", (double)want / rate,
                 (unsigned long)rate, (unsigned)(want * sizeof(int16_t)));
    afsk_modem_tap_start(src);
    size_t got = 0;
    int64_t deadline = esp_timer_get_time() + (int64_t)want * 1000000 / rate + 2000000;
    while (got < want && esp_timer_get_time() < deadline)
        got += afsk_modem_tap_read(buf + got, want - got, pdMS_TO_TICKS(200));
    afsk_modem_tap_stop();

    audio_status(line, line_size, "audio tap: recorded %u samples, sending", (unsigned)got);
    uint8_t hdr[44];
    wav_header(hdr, rate, got * sizeof(int16_t));
    bool ok = send_all(c, hdr, sizeof(hdr));
    for (size_t off = 0; ok && off < got; off += 1024)
        ok = send_all(c, buf + off, (got - off < 1024 ? got - off : 1024) * sizeof(int16_t));
    free(buf);
    return ok ? got : 0;
}

static void audio_task(void *arg)
{
    char peer[16], line[128];

    int ls = listen_on(s_audio_port);
    if (ls < 0)
    {
        ESP_LOGE(TAG, "audio: cannot listen on port %u", s_audio_port);
        vTaskDelete(NULL);
    }

    for (;;)
    {
        int c = accept_client(ls, peer, sizeof(peer));
        if (c < 0)
            continue;

        afsk_tap_t src = s_tap_src;
        uint32_t rate = afsk_modem_tap_rate(src);
        uint32_t capture_ms = s_tap_capture_ms;

        /* a buffered capture may take a while to send over a slow link: allow longer stalls */
        struct timeval tv = {.tv_sec = 15};
        setsockopt(c, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv));

        afsk_modem_stats_t st;
        afsk_modem_get_stats(&st);
        uint32_t dropped0 = st.tap_dropped;
        int64_t t0 = esp_timer_get_time();

        s_tap_busy = true;
        audio_status(line, sizeof(line), "audio tap: %s connected, %lu Hz %s", peer, (unsigned long)rate,
                     capture_ms ? "buffered" : "live");
        uint64_t total = capture_ms ? audio_capture(c, rate, src, capture_ms, line, sizeof(line))
                                    : audio_stream_live(c, rate, src);
        s_tap_busy = false;
        shutdown(c, SHUT_RDWR);
        close(c);

        afsk_modem_get_stats(&st);
        audio_status(line, sizeof(line), "audio tap: %s done after %.1f s, %llu samples sent, %lu bytes dropped", peer,
                     (esp_timer_get_time() - t0) / 1e6, (unsigned long long)total,
                     (unsigned long)(st.tap_dropped - dropped0));
    }
}

esp_err_t debug_server_start(uint16_t console_port, uint16_t audio_port)
{
    ESP_RETURN_ON_FALSE(!s_out, ESP_ERR_INVALID_STATE, TAG, "already started");
    s_console_port = console_port;
    s_audio_port = audio_port;

    s_out = xStreamBufferCreate(CONSOLE_OUT_BYTES, 1);
    s_out_lock = xSemaphoreCreateMutex();
    s_log_lock = xSemaphoreCreateMutex();
    ESP_RETURN_ON_FALSE(s_out && s_out_lock && s_log_lock, ESP_ERR_NO_MEM, TAG, "buffers");

    afsk_modem_set_debug_cb(on_modem_event, NULL);
    s_prev_vprintf = esp_log_set_vprintf(log_vprintf);

    /* core 0, away from the modem's RX/TX tasks; below the modem's priorities */
    ESP_RETURN_ON_FALSE(xTaskCreatePinnedToCore(console_task, "afsk_con", 4096, NULL, 4, NULL, 0) == pdPASS,
                        ESP_ERR_NO_MEM, TAG, "console task");
    ESP_RETURN_ON_FALSE(xTaskCreatePinnedToCore(audio_task, "afsk_tap", 4096, NULL, 4, NULL, 0) == pdPASS,
                        ESP_ERR_NO_MEM, TAG, "audio task");
    ESP_LOGI(TAG, "debug console on TCP %u, audio tap on TCP %u", console_port, audio_port);
    return ESP_OK;
}
