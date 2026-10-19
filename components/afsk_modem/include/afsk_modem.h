#pragma once
/*
 * 1200 baud AFSK (Bell 202) modem on the ESP32's own ADC and DAC.
 *
 * Receive: ADC1 runs in continuous (DMA) mode at 48 kHz and feeds afsk_demod. Every frame
 * with a good CRC goes to the rx callback.
 * Transmit: a 40 kHz GPTimer interrupt writes the DAC one sample at a time.
 *
 * Why a timer for the DAC: on the ESP32, ADC DMA and DAC DMA both need I2S0, so they can't
 * run together. With the DAC on a timer, RX keeps running while we transmit, which is also
 * what makes a plain DAC -> ADC loopback wire work as a test.
 *
 * Frames are just bytes. HDLC framing and the FCS are added and checked here; anything above
 * that (AX.25 or your own format) is up to the caller.
 */
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "driver/dac_types.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "hal/adc_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Called from the RX task for each frame with a good CRC (FCS already removed).
 * If the frame happens to be an AX.25 UI frame, ax25 points to it in TNC2 text form
 * ("SRC>DST,PATH:info"), otherwise it's NULL.
 */
typedef void (*afsk_modem_rx_cb_t)(const uint8_t *frame, int len, const char *ax25, void *ctx);

typedef struct
{
    adc_channel_t adc_channel; /* ADC1 channel for RX audio, ADC_CHANNEL_0 is GPIO36 */
    dac_channel_t dac_channel; /* DAC_CHAN_0 is GPIO25, DAC_CHAN_1 is GPIO26 */
    int ptt_gpio;              /* -1 if there's no PTT line */
    bool ptt_active_low;
    int tx_level;              /* DAC swing around mid-scale, 1..127 */
    int txdelay_ms;            /* flags sent after keying up, before the data */
    int txtail_flags;          /* flags sent after the data */
    int core;                  /* core for the RX/TX tasks and the DAC timer interrupt */
    afsk_modem_rx_cb_t rx_cb;
    void *rx_ctx;
} afsk_modem_config_t;

#define AFSK_MODEM_DEFAULT_CONFIG()  \
    {                                \
        .adc_channel = ADC_CHANNEL_0, \
        .dac_channel = DAC_CHAN_0,   \
        .ptt_gpio = -1,              \
        .ptt_active_low = true,      \
        .tx_level = 100,             \
        .txdelay_ms = 300,           \
        .txtail_flags = 3,           \
        .core = 1,                   \
        .rx_cb = NULL,               \
        .rx_ctx = NULL,              \
    }

esp_err_t afsk_modem_start(const afsk_modem_config_t *cfg);

/* ------------------------------------------------------------------ sending */

#define AFSK_MODEM_MAX_PAYLOAD 328 /* bytes per frame, not counting the FCS */

/* Queue a frame of 1..AFSK_MODEM_MAX_PAYLOAD bytes. Gives up after 100 ms if the queue is full. */
esp_err_t afsk_modem_send_raw(const uint8_t *data, int len);

/* Same, but you choose how long to wait for room in the queue. */
esp_err_t afsk_modem_send_raw_wait(const uint8_t *data, int len, TickType_t wait);

/* Convenience: build an AX.25 UI frame from TNC2 text and queue it. ESP_ERR_INVALID_ARG if it doesn't parse. */
esp_err_t afsk_modem_send(const char *tnc2);

/*
 * Channel access works like a KISS TNC: wait until there's no carrier, then transmit with
 * probability (persist + 1) / 256, or wait slot_ms and try again. Anything that got queued
 * in the meantime goes out in the same transmission, so it only pays the TX delay once.
 */
typedef struct
{
    int txdelay_ms;
    int persist;      /* 0..255 */
    int slot_ms;
    int txtail_flags;
    bool full_duplex; /* ignore carrier detect and just send */
} afsk_modem_channel_t;

void afsk_modem_get_channel(afsk_modem_channel_t *out);
void afsk_modem_set_channel(const afsk_modem_channel_t *in);

typedef enum
{
    AFSK_TONE_MARK,      /* steady 1200 Hz */
    AFSK_TONE_SPACE,     /* steady 2200 Hz */
    AFSK_TONE_ALTERNATE, /* switches between the two every bit */
} afsk_tone_t;

/* Key up and send a plain tone, handy for setting the radio's deviation or checking levels. */
esp_err_t afsk_modem_send_tone(afsk_tone_t tone, uint32_t ms);

/* ------------------------------------------------------------------ status */

typedef struct
{
    uint32_t frames_ok;
    uint32_t frames_bad_crc;
    uint32_t frames_sent;
    uint32_t adc_overflows;  /* the RX task fell behind the ADC DMA */
    uint32_t tap_dropped;    /* audio tap bytes lost because the reader was too slow */
    /* these are refreshed once a second */
    float sample_rate;       /* ADC samples actually received per second */
    uint16_t adc_min, adc_max;
    float adc_mean, adc_rms; /* ADC counts; rms is taken around the mean */
    float mark_level, space_level; /* tone amplitudes seen by the demod, fraction of full scale */
    uint32_t flags_per_sec;  /* HDLC flags decoded in the last second */
    uint32_t dcd_ms;         /* how long carrier was detected in the last second */
    float rx_cpu;            /* % of one core spent filtering and demodulating */
    /* these are live */
    bool transmitting;
    bool dcd;
} afsk_modem_stats_t;

void afsk_modem_get_stats(afsk_modem_stats_t *out);
bool afsk_modem_dcd(void); /* is there a carrier right now */

/* Copy the latest n raw 48 kHz ADC codes (n <= AFSK_MODEM_SCOPE_LEN), oldest first. */
#define AFSK_MODEM_SCOPE_LEN 4096
size_t afsk_modem_scope_read(uint16_t *out, size_t n);

/* ------------------------------------------------------------------ debugging */

typedef enum
{
    AFSK_DBG_FRAME_OK,  /* a decoded frame, with levels */
    AFSK_DBG_FRAME_BAD, /* something between two flags that failed the CRC, as hex */
    AFSK_DBG_TX,        /* transmit start and end */
    AFSK_DBG_LEVEL,     /* once a second: levels, flags, carrier */
} afsk_dbg_event_t;

/*
 * Gets one line of text per event (no newline), called from the RX and TX tasks.
 * Keep it short and never block in it. Pass NULL to stop.
 */
typedef void (*afsk_modem_debug_cb_t)(afsk_dbg_event_t type, const char *line, void *ctx);
void afsk_modem_set_debug_cb(afsk_modem_debug_cb_t cb, void *ctx);

/* Recent frames, good ones plus bad ones that had a real signal behind them. */
#define AFSK_MODEM_HISTORY_LEN 16
typedef struct
{
    int64_t time_us;   /* esp_timer time */
    bool crc_ok;
    int len;
    float mark_level, space_level;
    char text[192];    /* TNC2 for AX.25, otherwise "printable" |hex */
} afsk_modem_frame_rec_t;

/* Newest first. Returns how many were copied. */
int afsk_modem_get_history(afsk_modem_frame_rec_t *out, int max);

typedef enum
{
    AFSK_TAP_OFF,
    AFSK_TAP_RAW_48K,       /* ADC codes as int16, (code - 2048) * 16 */
    AFSK_TAP_DECIMATED_12K, /* what the demod works on: DC removed, 12 kHz, int16 full scale */
} afsk_tap_t;

/*
 * Audio tap: a copy of the RX audio for one reader. If the reader can't keep up,
 * samples are dropped (and counted in tap_dropped) rather than slowing down RX.
 */
esp_err_t afsk_modem_tap_start(afsk_tap_t source);
size_t afsk_modem_tap_read(int16_t *samples, size_t max_samples, TickType_t timeout);
void afsk_modem_tap_stop(void);
int afsk_modem_tap_rate(afsk_tap_t source);

#ifdef __cplusplus
}
#endif
