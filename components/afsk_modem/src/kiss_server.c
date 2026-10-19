#include "kiss_server.h"

#include <string.h>

#include "afsk_modem.h"

#include "esp_check.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/stream_buffer.h"
#include "freertos/task.h"
#include "lwip/sockets.h"

static const char TAG[] = "kiss";

#define FEND  0xC0
#define FESC  0xDB
#define TFEND 0xDC
#define TFESC 0xDD

#define KISS_DATA        0x00
#define KISS_TXDELAY     0x01
#define KISS_PERSIST     0x02
#define KISS_SLOTTIME    0x03
#define KISS_TXTAIL      0x04
#define KISS_FULLDUPLEX  0x05
#define KISS_SETHARDWARE 0x06
#define KISS_RETURN      0xFF

#define MAX_CLIENTS   3
#define RX_BUF_BYTES  8192 /* KISS-encoded frames waiting to go to the clients */

typedef struct
{
    int fd;
    char peer[16];
    bool in_frame, escape, overflow;
    int len;
    uint8_t buf[1 + AFSK_MODEM_MAX_PAYLOAD]; /* command byte + payload */
    int pending_len;                         /* > 0: data frame waiting for TX queue space */
    uint8_t rest[512];                       /* input received after the held-back frame */
    int rest_len;
} client_t;

static uint16_t s_port;
static StreamBufferHandle_t s_rx;
static volatile int s_nclients;
static kiss_server_stats_t s_stats;
static client_t s_clients[MAX_CLIENTS];

/* ------------------------------------------------------------------ air -> clients */

void kiss_server_frame_received(const uint8_t *frame, int len)
{
    static uint8_t enc[2 + 2 * (AFSK_MODEM_MAX_PAYLOAD + 2) + 1];
    if (!s_rx || s_nclients == 0)
        return;

    int n = 0;
    enc[n++] = FEND;
    enc[n++] = KISS_DATA; /* port 0 */
    for (int i = 0; i < len; i++)
    {
        if (frame[i] == FEND)
        {
            enc[n++] = FESC;
            enc[n++] = TFEND;
        }
        else if (frame[i] == FESC)
        {
            enc[n++] = FESC;
            enc[n++] = TFESC;
        }
        else
            enc[n++] = frame[i];
    }
    enc[n++] = FEND;

    /* whole frames only, so a slow client never sees half a frame */
    if (xStreamBufferSpacesAvailable(s_rx) < (size_t)n)
    {
        s_stats.dropped++;
        return;
    }
    xStreamBufferSend(s_rx, enc, n, 0);
    s_stats.from_air++;
}

/* ------------------------------------------------------------------ clients -> air */

static void apply_command(client_t *c, uint8_t cmd, uint8_t value)
{
    afsk_modem_channel_t ch;
    afsk_modem_get_channel(&ch);
    switch (cmd)
    {
    case KISS_TXDELAY:
        ch.txdelay_ms = value * 10;
        break;
    case KISS_PERSIST:
        ch.persist = value;
        break;
    case KISS_SLOTTIME:
        ch.slot_ms = value * 10;
        break;
    case KISS_TXTAIL:
        ch.txtail_flags = (value * 10 * 1200 / 1000 + 7) / 8;
        break;
    case KISS_FULLDUPLEX:
        ch.full_duplex = value != 0;
        break;
    default:
        return; /* SetHardware and unknown commands: ignored */
    }
    afsk_modem_set_channel(&ch);
    afsk_modem_get_channel(&ch);
    ESP_LOGI(TAG, "%s: cmd %u = %u -> txdelay %d ms, persist %d, slot %d ms, tail %d flags, %s", c->peer, cmd, value,
             ch.txdelay_ms, ch.persist, ch.slot_ms, ch.txtail_flags, ch.full_duplex ? "full duplex" : "half duplex");
}

/* A complete KISS frame (between FENDs, unescaped) arrived in c->buf. */
static void handle_frame(client_t *c)
{
    if (c->overflow || c->len < 1)
    {
        if (c->overflow)
            s_stats.bad_kiss++;
        return;
    }
    uint8_t cmd = c->buf[0];
    if (cmd == KISS_RETURN)
        return;
    if ((cmd >> 4) != 0)
        return; /* only port 0 exists */

    if ((cmd & 0x0F) == KISS_DATA)
    {
        if (c->len < 2)
            return;
        if (afsk_modem_send_raw_wait(c->buf + 1, c->len - 1, 0) == ESP_OK)
            s_stats.to_air++;
        else
            c->pending_len = c->len; /* TX queue full: hold it and stop reading this client */
    }
    else if (c->len >= 2)
    {
        apply_command(c, cmd & 0x0F, c->buf[1]);
    }
}

/* Feed received bytes; stops early if a data frame had to be held back (returns bytes consumed). */
static int kiss_decode(client_t *c, const uint8_t *data, int n)
{
    for (int i = 0; i < n; i++)
    {
        uint8_t b = data[i];
        if (b == FEND)
        {
            if (c->in_frame && c->len > 0)
            {
                handle_frame(c);
                c->len = 0;
                c->overflow = false;
                c->escape = false;
                if (c->pending_len)
                    return i + 1;
            }
            c->in_frame = true;
            continue;
        }
        if (!c->in_frame)
            continue;
        if (c->escape)
        {
            b = b == TFEND ? FEND : b == TFESC ? FESC : b;
            c->escape = false;
        }
        else if (b == FESC)
        {
            c->escape = true;
            continue;
        }
        if (c->len < (int)sizeof(c->buf))
            c->buf[c->len++] = b;
        else
            c->overflow = true;
    }
    return n;
}

/* ------------------------------------------------------------------ task */

static void drop_client(client_t *c)
{
    ESP_LOGI(TAG, "client %s disconnected", c->peer);
    close(c->fd);
    c->fd = -1;
    s_nclients--;
}

static bool send_all(int fd, const uint8_t *p, size_t len)
{
    while (len)
    {
        int n = send(fd, p, len, 0);
        if (n <= 0)
            return false;
        p += n;
        len -= n;
    }
    return true;
}

static void kiss_task(void *arg)
{
    static uint8_t tmp[512];

    int ls = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    int one = 1;
    setsockopt(ls, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));
    struct sockaddr_in addr = {.sin_family = AF_INET, .sin_port = htons(s_port), .sin_addr.s_addr = htonl(INADDR_ANY)};
    if (ls < 0 || bind(ls, (struct sockaddr *)&addr, sizeof(addr)) < 0 || listen(ls, 2) < 0)
    {
        ESP_LOGE(TAG, "cannot listen on port %u", s_port);
        vTaskDelete(NULL);
    }
    for (int i = 0; i < MAX_CLIENTS; i++)
        s_clients[i].fd = -1;
    ESP_LOGI(TAG, "KISS over TCP on port %u", s_port);

    for (;;)
    {
        fd_set rfds;
        FD_ZERO(&rfds);
        FD_SET(ls, &rfds);
        int maxfd = ls;
        for (int i = 0; i < MAX_CLIENTS; i++)
        {
            client_t *c = &s_clients[i];
            if (c->fd >= 0 && !c->pending_len)
            {
                FD_SET(c->fd, &rfds);
                if (c->fd > maxfd)
                    maxfd = c->fd;
            }
        }
        struct timeval tv = {.tv_sec = 0, .tv_usec = 20000};
        int ready = select(maxfd + 1, &rfds, NULL, NULL, &tv);

        if (ready > 0 && FD_ISSET(ls, &rfds))
        {
            struct sockaddr_in from;
            socklen_t flen = sizeof(from);
            int fd = accept(ls, (struct sockaddr *)&from, &flen);
            client_t *slot = NULL;
            for (int i = 0; i < MAX_CLIENTS && fd >= 0; i++)
                if (s_clients[i].fd < 0)
                {
                    slot = &s_clients[i];
                    break;
                }
            if (fd >= 0 && !slot)
            {
                ESP_LOGW(TAG, "too many clients, refusing one");
                close(fd);
            }
            else if (slot)
            {
                memset(slot, 0, sizeof(*slot));
                slot->fd = fd;
                inet_ntoa_r(from.sin_addr, slot->peer, sizeof(slot->peer));
                setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &one, sizeof(one));
                struct timeval stv = {.tv_sec = 5};
                setsockopt(fd, SOL_SOCKET, SO_SNDTIMEO, &stv, sizeof(stv));
                if (s_nclients++ == 0)
                    xStreamBufferReset(s_rx); /* nothing stale for the first client */
                ESP_LOGI(TAG, "client %s connected", slot->peer);
            }
        }

        for (int i = 0; i < MAX_CLIENTS; i++)
        {
            client_t *c = &s_clients[i];
            if (c->fd < 0)
                continue;

            /*
             * A data frame is held back while the TX queue is full; meanwhile this client is not read,
             * so TCP pushes back on the sender. Retry it, then continue with the input that followed it.
             */
            if (c->pending_len)
            {
                if (afsk_modem_send_raw_wait(c->buf + 1, c->pending_len - 1, 0) != ESP_OK)
                    continue;
                s_stats.to_air++;
                c->pending_len = 0;
                int done = kiss_decode(c, c->rest, c->rest_len);
                memmove(c->rest, c->rest + done, c->rest_len - done);
                c->rest_len -= done;
                continue;
            }

            if (ready > 0 && FD_ISSET(c->fd, &rfds))
            {
                /* rest_len is 0 here: a client with leftovers is pending, or just drained them */
                int n = recv(c->fd, c->rest, sizeof(c->rest), 0);
                if (n <= 0)
                {
                    drop_client(c);
                    continue;
                }
                int done = kiss_decode(c, c->rest, n);
                memmove(c->rest, c->rest + done, n - done);
                c->rest_len = n - done;
            }
        }

        /* air -> all clients */
        size_t n;
        while ((n = xStreamBufferReceive(s_rx, tmp, sizeof(tmp), 0)) > 0)
            for (int i = 0; i < MAX_CLIENTS; i++)
                if (s_clients[i].fd >= 0 && !send_all(s_clients[i].fd, tmp, n))
                    drop_client(&s_clients[i]);
        s_stats.clients = s_nclients;
    }
}

esp_err_t kiss_server_start(uint16_t port)
{
    ESP_RETURN_ON_FALSE(!s_rx, ESP_ERR_INVALID_STATE, TAG, "already started");
    s_port = port;
    s_rx = xStreamBufferCreate(RX_BUF_BYTES, 1);
    ESP_RETURN_ON_FALSE(s_rx, ESP_ERR_NO_MEM, TAG, "buffer");
    ESP_RETURN_ON_FALSE(xTaskCreatePinnedToCore(kiss_task, "kiss", 4096, NULL, 5, NULL, 0) == pdPASS, ESP_ERR_NO_MEM,
                        TAG, "task");
    return ESP_OK;
}

void kiss_server_get_stats(kiss_server_stats_t *out)
{
    *out = s_stats;
    out->clients = s_nclients;
}
