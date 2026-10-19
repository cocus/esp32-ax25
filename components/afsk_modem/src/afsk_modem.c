#include "afsk_modem.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

#include "afsk_demod.h"
#include "afsk_mod.h"
#include "ax25.h"

#include "driver/dac_oneshot.h"
#include "driver/gpio.h"
#include "driver/gptimer.h"
#include "esp_adc/adc_continuous.h"
#include "esp_attr.h"
#include "esp_check.h"
#include "esp_log.h"
#include "esp_random.h"
#include "esp_timer.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/stream_buffer.h"
#include "freertos/task.h"
#include "hal/dac_ll.h"

static const char TAG[] = "afsk_modem";

#define TX_SAMPLE_RATE    40000 /* 1 MHz timer / 25 */
#define TX_QUEUE_LEN      8
#define ADC_FRAME_BYTES   1024  /* 512 samples, ~10.7 ms */
#define ADC_FRAME_SAMPLES (ADC_FRAME_BYTES / sizeof(adc_digi_output_data_t))
#define TAP_BUFFER_BYTES  (32 * 1024)
#define SERIAL_STATS_SECONDS 10

typedef struct
{
    enum { TX_FRAME, TX_TONE } kind;
    afsk_tone_t tone;
    uint32_t tone_ms;
    int len;
    uint8_t frame[AX25_MAX_FRAME];
} tx_item_t;

static afsk_modem_config_t s_cfg;
static afsk_demod_t s_demod;
static afsk_demod_trace_t s_trace;
static afsk_modem_stats_t s_stats;
static volatile uint32_t s_adc_overflows;

static afsk_modem_debug_cb_t s_dbg_cb;
static void *s_dbg_ctx;

static SemaphoreHandle_t s_hist_lock;
static afsk_modem_frame_rec_t s_hist[AFSK_MODEM_HISTORY_LEN];
static int s_hist_head, s_hist_count;

static StreamBufferHandle_t s_tap;

static uint16_t s_scope[AFSK_MODEM_SCOPE_LEN]; /* power of two */
static volatile uint32_t s_scope_pos;
static volatile afsk_tap_t s_tap_src;

static QueueHandle_t s_tx_queue;
static SemaphoreHandle_t s_tx_done;
static SemaphoreHandle_t s_send_lock;
static gptimer_handle_t s_tx_timer;
static dac_oneshot_handle_t s_dac;
static afsk_mod_t s_mod;
static volatile bool s_tx_active;
/* worst case: 1 s of preamble flags + max frame fully bit-stuffed + tail */
/* one burst: up to ~10 s on air (preamble + several frames + tail) */
static uint8_t s_tx_bits[1536];
static afsk_modem_channel_t s_chan;

static void format_raw(const uint8_t *data, int len, char *out, size_t size);

static void dbg_emit(afsk_dbg_event_t type, const char *line)
{
    afsk_modem_debug_cb_t cb = s_dbg_cb;
    if (cb)
        cb(type, line, s_dbg_ctx);
}

void afsk_modem_set_debug_cb(afsk_modem_debug_cb_t cb, void *ctx)
{
    s_dbg_cb = NULL;
    s_dbg_ctx = ctx;
    s_dbg_cb = cb;
}

/* ------------------------------------------------------------------ TX */

static void ptt_set(bool on)
{
    if (s_cfg.ptt_gpio >= 0)
        gpio_set_level(s_cfg.ptt_gpio, on ^ s_cfg.ptt_active_low);
}

/* Integer only: the FPU must not be used from ISRs on Xtensa. */
static bool IRAM_ATTR tx_timer_isr(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *ctx)
{
    uint8_t sample = 128;
    if (afsk_mod_next(&s_mod, &sample))
    {
        dac_ll_update_output_value(s_cfg.dac_channel, sample);
        return false;
    }

    dac_ll_update_output_value(s_cfg.dac_channel, 128);
    if (s_tx_active)
    {
        s_tx_active = false;
        BaseType_t woken = pdFALSE;
        xSemaphoreGiveFromISR(s_tx_done, &woken);
        return woken == pdTRUE;
    }
    return false;
}

static esp_err_t tx_hw_init(void)
{
    if (s_cfg.ptt_gpio >= 0)
    {
        gpio_config_t io = {
            .pin_bit_mask = 1ULL << s_cfg.ptt_gpio,
            .mode = GPIO_MODE_OUTPUT,
        };
        ESP_RETURN_ON_ERROR(gpio_config(&io), TAG, "ptt gpio");
        ptt_set(false);
    }

    dac_oneshot_config_t dac_cfg = {.chan_id = s_cfg.dac_channel};
    ESP_RETURN_ON_ERROR(dac_oneshot_new_channel(&dac_cfg, &s_dac), TAG, "dac");
    dac_oneshot_output_voltage(s_dac, 128);

    afsk_mod_init(&s_mod, TX_SAMPLE_RATE, s_cfg.tx_level);

    gptimer_config_t tcfg = {
        .clk_src = GPTIMER_CLK_SRC_DEFAULT,
        .direction = GPTIMER_COUNT_UP,
        .resolution_hz = 1000000,
    };
    ESP_RETURN_ON_ERROR(gptimer_new_timer(&tcfg, &s_tx_timer), TAG, "gptimer");
    gptimer_alarm_config_t alarm = {
        .alarm_count = 1000000 / TX_SAMPLE_RATE,
        .reload_count = 0,
        .flags.auto_reload_on_alarm = true,
    };
    ESP_RETURN_ON_ERROR(gptimer_set_alarm_action(s_tx_timer, &alarm), TAG, "alarm");
    gptimer_event_callbacks_t cbs = {.on_alarm = tx_timer_isr};
    ESP_RETURN_ON_ERROR(gptimer_register_event_callbacks(s_tx_timer, &cbs, NULL), TAG, "callbacks");
    ESP_RETURN_ON_ERROR(gptimer_enable(s_tx_timer), TAG, "enable");
    return ESP_OK;
}

static bool channel_busy(void)
{
    return s_demod.dcd;
}

/*
 * KISS-style p-persistent CSMA: wait for a clear channel, then go with probability (persist+1)/256 per slot.
 * Reports how long the carrier kept us waiting and how many random slots were lost, for the TX log.
 */
static void wait_for_channel(int *busy_ms, int *slots_lost)
{
    *busy_ms = *slots_lost = 0;
    if (s_chan.full_duplex)
        return;
    for (;;)
    {
        while (channel_busy())
        {
            vTaskDelay(pdMS_TO_TICKS(10));
            *busy_ms += 10;
        }
        if ((int)(esp_random() & 0xFF) <= s_chan.persist)
            return;
        (*slots_lost)++;
        vTaskDelay(pdMS_TO_TICKS(s_chan.slot_ms > 10 ? s_chan.slot_ms : 10));
    }
}

/*
 * Append one frame (and a separating flag) if it still leaves room for the tail flags.
 * A whole number of flags leaves the NRZI level unchanged (two zeros per flag), so the trial
 * tail can be dropped again just by rewinding nbits.
 */
static bool encode_append(afsk_encoder_t *e, const uint8_t *frame, int len, bool first, int tail_flags)
{
    afsk_encoder_t trial = *e;
    if ((!first && afsk_enc_flags(&trial, 1) < 0) || afsk_enc_frame(&trial, frame, len) < 0 ||
        afsk_enc_flags(&trial, tail_flags) < 0)
        return false;
    trial.nbits -= tail_flags * 8;
    *e = trial;
    return true;
}

static void tx_task(void *arg)
{
    static tx_item_t item;
    static char line[AX25_MAX_FRAME * 6 + 64];
    static const uint8_t tone_bits[] = {
        [AFSK_TONE_MARK] = 0xFF,
        [AFSK_TONE_SPACE] = 0x00,
        [AFSK_TONE_ALTERNATE] = 0x55,
    };
    static const char *tone_names[] = {"mark 1200 Hz", "space 2200 Hz", "alternating"};

    /* Created on s_cfg.core, so the timer interrupt is allocated there too. */
    esp_err_t err = tx_hw_init();
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "TX init failed: %s", esp_err_to_name(err));
        vTaskDelete(NULL);
    }

    for (;;)
    {
        xQueueReceive(s_tx_queue, &item, portMAX_DELAY);

        uint32_t duration_ms;
        int nframes = 0;
        int64_t t_wait = esp_timer_get_time();
        if (item.kind == TX_TONE)
        {
            afsk_mod_load_loop(&s_mod, &tone_bits[item.tone], 8, item.tone_ms * (TX_SAMPLE_RATE / 1000));
            duration_ms = item.tone_ms;
            snprintf(line, sizeof(line), "TX tone %s for %lu ms", tone_names[item.tone], (unsigned long)item.tone_ms);
            dbg_emit(AFSK_DBG_TX, line);
        }
        else
        {
            int busy_ms, slots_lost;
            wait_for_channel(&busy_ms, &slots_lost); /* frames queued meanwhile go out in the same burst */
            int waited_ms = (int)((esp_timer_get_time() - t_wait) / 1000);

            int preamble = (s_chan.txdelay_ms * 1200 / 1000 + 7) / 8;
            int tail = s_chan.txtail_flags;
            afsk_encoder_t e;
            afsk_enc_init(&e, s_tx_bits, sizeof(s_tx_bits));
            afsk_enc_flags(&e, preamble);
            if (!encode_append(&e, item.frame, item.len, true, tail))
            {
                ESP_LOGE(TAG, "TX bit buffer too small");
                continue;
            }
            nframes = 1;
            int n = snprintf(line, sizeof(line), "TX %d bytes: ", item.len - 2);
            if (ax25_to_tnc2(item.frame, item.len - 2, line + n, sizeof(line) - n) < 0)
                format_raw(item.frame, item.len - 2, line + n, sizeof(line) - n);
            dbg_emit(AFSK_DBG_TX, line);

            /* bundle whatever else is waiting, as long as it fits */
            while (xQueuePeek(s_tx_queue, &item, 0) == pdTRUE && item.kind == TX_FRAME &&
                   encode_append(&e, item.frame, item.len, false, tail))
            {
                xQueueReceive(s_tx_queue, &item, 0);
                nframes++;
                n = snprintf(line, sizeof(line), "TX +%d bytes: ", item.len - 2);
                if (ax25_to_tnc2(item.frame, item.len - 2, line + n, sizeof(line) - n) < 0)
                    format_raw(item.frame, item.len - 2, line + n, sizeof(line) - n);
                dbg_emit(AFSK_DBG_TX, line);
            }
            afsk_enc_flags(&e, tail); /* room was reserved by encode_append */

            afsk_mod_load(&s_mod, s_tx_bits, e.nbits);
            duration_ms = e.nbits * 1000 / 1200;
            snprintf(line, sizeof(line),
                     "TX burst: %d frame(s), %d bits, %lu ms on air, channel wait %d ms (carrier %d ms, %d slot(s) lost)",
                     nframes, e.nbits, (unsigned long)duration_ms, waited_ms, busy_ms, slots_lost);
            dbg_emit(AFSK_DBG_TX, line);
        }

        xSemaphoreTake(s_tx_done, 0);
        s_tx_active = true;
        s_stats.transmitting = true;

        int64_t t0 = esp_timer_get_time();
        ptt_set(true);
        gptimer_set_raw_count(s_tx_timer, 0);
        gptimer_start(s_tx_timer);

        bool finished = xSemaphoreTake(s_tx_done, pdMS_TO_TICKS(duration_ms + 500)) == pdTRUE;

        gptimer_stop(s_tx_timer);
        s_tx_active = false;
        dac_oneshot_output_voltage(s_dac, 128);
        ptt_set(false);
        s_stats.transmitting = false;
        s_stats.frames_sent += nframes;

        snprintf(line, sizeof(line), "TX %s after %lld ms", finished ? "done" : "TIMED OUT",
                 (long long)((esp_timer_get_time() - t0) / 1000));
        dbg_emit(AFSK_DBG_TX, line);
        if (!finished)
            ESP_LOGW(TAG, "%s", line);
    }
}

esp_err_t afsk_modem_send_raw_wait(const uint8_t *data, int len, TickType_t wait)
{
    static tx_item_t item; /* guarded by s_send_lock */
    if (!s_tx_queue)
        return ESP_ERR_INVALID_STATE;
    if (!data || len < 1 || len > AFSK_MODEM_MAX_PAYLOAD)
        return ESP_ERR_INVALID_ARG;

    xSemaphoreTake(s_send_lock, portMAX_DELAY);
    item.kind = TX_FRAME;
    memcpy(item.frame, data, len);
    uint16_t fcs = ax25_crc(data, len);
    item.frame[len] = fcs & 0xFF;
    item.frame[len + 1] = fcs >> 8;
    item.len = len + 2;
    esp_err_t ret = xQueueSend(s_tx_queue, &item, wait) == pdTRUE ? ESP_OK : ESP_ERR_TIMEOUT;
    xSemaphoreGive(s_send_lock);
    return ret;
}

esp_err_t afsk_modem_send_raw(const uint8_t *data, int len)
{
    return afsk_modem_send_raw_wait(data, len, pdMS_TO_TICKS(100));
}

esp_err_t afsk_modem_send(const char *tnc2)
{
    static tx_item_t item; /* too big for most callers' stacks; guarded by s_send_lock */
    if (!s_tx_queue)
        return ESP_ERR_INVALID_STATE;

    xSemaphoreTake(s_send_lock, portMAX_DELAY);
    item.kind = TX_FRAME;
    item.len = ax25_from_tnc2(tnc2, item.frame, sizeof(item.frame));
    esp_err_t ret = ESP_ERR_INVALID_ARG;
    if (item.len > 0)
        ret = xQueueSend(s_tx_queue, &item, pdMS_TO_TICKS(100)) == pdTRUE ? ESP_OK : ESP_ERR_TIMEOUT;
    xSemaphoreGive(s_send_lock);
    return ret;
}

esp_err_t afsk_modem_send_tone(afsk_tone_t tone, uint32_t ms)
{
    static tx_item_t item;
    if (!s_tx_queue)
        return ESP_ERR_INVALID_STATE;
    if (tone > AFSK_TONE_ALTERNATE || ms == 0 || ms > 60000)
        return ESP_ERR_INVALID_ARG;

    xSemaphoreTake(s_send_lock, portMAX_DELAY);
    item.kind = TX_TONE;
    item.tone = tone;
    item.tone_ms = ms;
    esp_err_t ret = xQueueSend(s_tx_queue, &item, pdMS_TO_TICKS(100)) == pdTRUE ? ESP_OK : ESP_ERR_TIMEOUT;
    xSemaphoreGive(s_send_lock);
    return ret;
}

/* ------------------------------------------------------------------ RX */

static float to_db(float ratio)
{
    return ratio > 0.0f ? 20.0f * log10f(ratio) : -99.0f;
}

/* "text with dots for binary" | 0123ABCD... */
static void format_raw(const uint8_t *data, int len, char *out, size_t size)
{
    size_t n = 0;
    out[n++] = '"';
    for (int i = 0; i < len && n < size / 3; i++)
        out[n++] = (data[i] >= 0x20 && data[i] < 0x7F) ? (char)data[i] : '.';
    n += snprintf(out + n, size - n, "\" |");
    for (int i = 0; i < len && n < size - 3; i++)
        n += snprintf(out + n, size - n, "%02X", data[i]);
    out[n < size ? n : size - 1] = '\0';
}

/* Bad-CRC candidates go into the history only with a real signal present, not idle noise. */
#define HISTORY_BAD_MIN_LEVEL 0.05f

static void on_frame(const uint8_t *frame, int len, const afsk_frame_info_t *info, void *ctx)
{
    static char ax25[AX25_MAX_FRAME * 6];
    static char text[AX25_MAX_FRAME * 6];
    static char line[sizeof(text) + 96];

    bool ok = info->crc_ok;
    bool is_ax25 = ok && ax25_to_tnc2(frame, len, ax25, sizeof(ax25)) >= 0;
    if (is_ax25)
        strlcpy(text, ax25, sizeof(text));
    else
        format_raw(frame, len, text, sizeof(text));

    float level = info->mark_level > info->space_level ? info->mark_level : info->space_level;
    if (ok || level >= HISTORY_BAD_MIN_LEVEL)
    {
        xSemaphoreTake(s_hist_lock, portMAX_DELAY);
        afsk_modem_frame_rec_t *rec = &s_hist[s_hist_head];
        rec->time_us = esp_timer_get_time();
        rec->crc_ok = ok;
        rec->len = len;
        rec->mark_level = info->mark_level;
        rec->space_level = info->space_level;
        strlcpy(rec->text, text, sizeof(rec->text));
        s_hist_head = (s_hist_head + 1) % AFSK_MODEM_HISTORY_LEN;
        if (s_hist_count < AFSK_MODEM_HISTORY_LEN)
            s_hist_count++;
        xSemaphoreGive(s_hist_lock);
    }

    snprintf(line, sizeof(line), "%s %d bytes, mark %.3f space %.3f FS (twist %+.1f dB): %s",
             ok ? "RX" : "RX BAD CRC", len, info->mark_level, info->space_level,
             to_db(info->space_level / info->mark_level), text);
    dbg_emit(ok ? AFSK_DBG_FRAME_OK : AFSK_DBG_FRAME_BAD, line);

    if (!ok)
        return;
    if (s_cfg.rx_cb)
        s_cfg.rx_cb(frame, len, is_ax25 ? ax25 : NULL, s_cfg.rx_ctx);
    else
        ESP_LOGI(TAG, "RX %s", text);
}

int afsk_modem_get_history(afsk_modem_frame_rec_t *out, int max)
{
    if (!s_hist_lock)
        return 0;
    xSemaphoreTake(s_hist_lock, portMAX_DELAY);
    int n = s_hist_count < max ? s_hist_count : max;
    for (int i = 0; i < n; i++)
        out[i] = s_hist[(s_hist_head - 1 - i + AFSK_MODEM_HISTORY_LEN) % AFSK_MODEM_HISTORY_LEN];
    xSemaphoreGive(s_hist_lock);
    return n;
}

static bool IRAM_ATTR adc_pool_ovf(adc_continuous_handle_t handle, const adc_continuous_evt_data_t *edata, void *ctx)
{
    s_adc_overflows++;
    return false;
}

int afsk_modem_tap_rate(afsk_tap_t source)
{
    return source == AFSK_TAP_DECIMATED_12K ? AFSK_DEMOD_FS : AFSK_DEMOD_FS_IN;
}

esp_err_t afsk_modem_tap_start(afsk_tap_t source)
{
    if (!s_tap)
        s_tap = xStreamBufferCreate(TAP_BUFFER_BYTES, 1);
    ESP_RETURN_ON_FALSE(s_tap, ESP_ERR_NO_MEM, TAG, "tap buffer");
    s_tap_src = AFSK_TAP_OFF;
    vTaskDelay(pdMS_TO_TICKS(30)); /* let the RX task finish any write in progress */
    xStreamBufferReset(s_tap);
    s_tap_src = source;
    return ESP_OK;
}

size_t afsk_modem_tap_read(int16_t *samples, size_t max_samples, TickType_t timeout)
{
    if (!s_tap)
        return 0;
    return xStreamBufferReceive(s_tap, samples, max_samples * sizeof(int16_t), timeout) / sizeof(int16_t);
}

void afsk_modem_tap_stop(void)
{
    s_tap_src = AFSK_TAP_OFF;
}

static void tap_write(const int16_t *samples, int n)
{
    size_t bytes = n * sizeof(int16_t);
    size_t sent = xStreamBufferSend(s_tap, samples, bytes, 0);
    if (sent < bytes)
        s_stats.tap_dropped += bytes - sent;
}

static void rx_task(void *arg)
{
    static uint8_t buf[ADC_FRAME_BYTES];
    static int16_t tap_buf[ADC_FRAME_SAMPLES];
    static char line[160];
    adc_continuous_handle_t adc;

    adc_continuous_handle_cfg_t hcfg = {
        .max_store_buf_size = ADC_FRAME_BYTES * 8,
        .conv_frame_size = ADC_FRAME_BYTES,
    };
    ESP_ERROR_CHECK(adc_continuous_new_handle(&hcfg, &adc));

    adc_digi_pattern_config_t pattern = {
        .atten = ADC_ATTEN_DB_12,
        .channel = s_cfg.adc_channel,
        .unit = ADC_UNIT_1,
        .bit_width = SOC_ADC_DIGI_MAX_BITWIDTH,
    };
    adc_continuous_config_t acfg = {
        .pattern_num = 1,
        .adc_pattern = &pattern,
        .sample_freq_hz = AFSK_DEMOD_FS_IN,
        .conv_mode = ADC_CONV_SINGLE_UNIT_1,
    };
    ESP_ERROR_CHECK(adc_continuous_config(adc, &acfg));
    adc_continuous_evt_cbs_t cbs = {.on_pool_ovf = adc_pool_ovf};
    ESP_ERROR_CHECK(adc_continuous_register_event_callbacks(adc, &cbs, NULL));
    ESP_ERROR_CHECK(adc_continuous_start(adc));

    int io = -1;
    adc_continuous_channel_to_io(ADC_UNIT_1, s_cfg.adc_channel, &io);
    ESP_LOGI(TAG, "RX on ADC1 ch%d (GPIO%d) @ %d Hz", s_cfg.adc_channel, io, AFSK_DEMOD_FS_IN);

    int64_t t0 = esp_timer_get_time();
    uint32_t nsamples = 0, prev_flags = 0, dcd_samples = 0;
    int64_t busy_us = 0;
    uint16_t vmin = 0xFFFF, vmax = 0;
    double sum = 0.0, sumsq = 0.0;
    int seconds = 0;

    for (;;)
    {
        uint32_t len = 0;
        if (adc_continuous_read(adc, buf, sizeof(buf), &len, ADC_MAX_DELAY) != ESP_OK)
            continue;

        int64_t t_block = esp_timer_get_time();
        afsk_tap_t tap = s_tap_src;
        int ntap = 0;
        const adc_digi_output_data_t *p = (const adc_digi_output_data_t *)buf;
        int n = len / sizeof(adc_digi_output_data_t);
        for (int i = 0; i < n; i++)
        {
            uint16_t v = p[i].type1.data;
            s_scope[s_scope_pos++ & (AFSK_MODEM_SCOPE_LEN - 1)] = v;
            if (v < vmin)
                vmin = v;
            if (v > vmax)
                vmax = v;
            sum += v;
            sumsq += (double)v * v;

            bool out12k = afsk_demod_process(&s_demod, (v - 2048) * (1.0f / 2048.0f));
            if (out12k)
                dcd_samples += s_demod.dcd;

            if (tap == AFSK_TAP_RAW_48K)
            {
                tap_buf[ntap++] = (int16_t)((v - 2048) * 16);
            }
            else if (tap == AFSK_TAP_DECIMATED_12K && out12k)
            {
                float s = s_trace.in * 32768.0f;
                tap_buf[ntap++] = (int16_t)(s > 32767.0f ? 32767 : s < -32768.0f ? -32768 : s);
            }
        }
        if (ntap)
            tap_write(tap_buf, ntap);
        nsamples += n;

        int64_t now = esp_timer_get_time();
        busy_us += now - t_block;

        if (now - t0 < 1000000)
            continue;

        /* once per second */
        double mean = sum / nsamples;
        double var = sumsq / nsamples - mean * mean;
        s_stats.sample_rate = nsamples * 1e6f / (float)(now - t0);
        s_stats.adc_min = vmin;
        s_stats.adc_max = vmax;
        s_stats.adc_mean = (float)mean;
        s_stats.adc_rms = (float)sqrt(var > 0.0 ? var : 0.0);
        s_stats.mark_level = afsk_demod_mark_level(&s_demod);
        s_stats.space_level = afsk_demod_space_level(&s_demod);
        s_stats.flags_per_sec = s_demod.flags_seen - prev_flags;
        s_stats.frames_ok = s_demod.frames_ok;
        s_stats.frames_bad_crc = s_demod.frames_bad_crc;
        s_stats.adc_overflows = s_adc_overflows;
        s_stats.dcd_ms = dcd_samples * 1000 / AFSK_DEMOD_FS;
        s_stats.rx_cpu = busy_us * 100.0f / (float)(now - t0);
        prev_flags = s_demod.flags_seen;
        dcd_samples = 0;
        busy_us = 0;

        snprintf(line, sizeof(line),
                 "level: ADC %u..%u mean %.0f rms %.1f | mark %.3f space %.3f FS | flags/s %lu | DCD %lu ms | %.0f S/s%s",
                 vmin, vmax, s_stats.adc_mean, s_stats.adc_rms, s_stats.mark_level, s_stats.space_level,
                 (unsigned long)s_stats.flags_per_sec, (unsigned long)s_stats.dcd_ms, s_stats.sample_rate,
                 (vmin == 0 || vmax == 4095) ? " | CLIPPING" : "");
        dbg_emit(AFSK_DBG_LEVEL, line);

        if (++seconds >= SERIAL_STATS_SECONDS)
        {
            seconds = 0;
            ESP_LOGI(TAG, "%s | frames ok %lu bad %lu sent %lu | overruns %lu", line,
                     (unsigned long)s_stats.frames_ok, (unsigned long)s_stats.frames_bad_crc,
                     (unsigned long)s_stats.frames_sent, (unsigned long)s_stats.adc_overflows);
        }

        t0 = now;
        nsamples = 0;
        vmin = 0xFFFF;
        vmax = 0;
        sum = sumsq = 0.0;
    }
}

/* ------------------------------------------------------------------ */

esp_err_t afsk_modem_start(const afsk_modem_config_t *cfg)
{
    ESP_RETURN_ON_FALSE(!s_tx_queue, ESP_ERR_INVALID_STATE, TAG, "already started");
    s_cfg = *cfg;
    s_chan = (afsk_modem_channel_t){
        .txdelay_ms = cfg->txdelay_ms,
        .persist = 63,
        .slot_ms = 100,
        .txtail_flags = cfg->txtail_flags,
        .full_duplex = false,
    };

    afsk_demod_init(&s_demod, on_frame, NULL);
    s_demod.trace = &s_trace;

    s_tx_queue = xQueueCreate(TX_QUEUE_LEN, sizeof(tx_item_t));
    s_tx_done = xSemaphoreCreateBinary();
    s_send_lock = xSemaphoreCreateMutex();
    s_hist_lock = xSemaphoreCreateMutex();
    ESP_RETURN_ON_FALSE(s_tx_queue && s_tx_done && s_send_lock && s_hist_lock, ESP_ERR_NO_MEM, TAG, "queue");

    ESP_RETURN_ON_FALSE(xTaskCreatePinnedToCore(rx_task, "afsk_rx", 6144, NULL, 10, NULL, s_cfg.core) == pdPASS,
                        ESP_ERR_NO_MEM, TAG, "rx task");
    ESP_RETURN_ON_FALSE(xTaskCreatePinnedToCore(tx_task, "afsk_tx", 6144, NULL, 11, NULL, s_cfg.core) == pdPASS,
                        ESP_ERR_NO_MEM, TAG, "tx task");
    return ESP_OK;
}

size_t afsk_modem_scope_read(uint16_t *out, size_t n)
{
    if (n > AFSK_MODEM_SCOPE_LEN)
        n = AFSK_MODEM_SCOPE_LEN;
    /* Lock-free: the RX task keeps writing; at worst the oldest few samples are newer than expected. */
    uint32_t end = s_scope_pos;
    for (size_t i = 0; i < n; i++)
        out[i] = s_scope[(end - n + i) & (AFSK_MODEM_SCOPE_LEN - 1)];
    return n;
}

void afsk_modem_get_stats(afsk_modem_stats_t *out)
{
    *out = s_stats;
    out->dcd = s_demod.dcd;
}

bool afsk_modem_dcd(void)
{
    return s_demod.dcd;
}

void afsk_modem_get_channel(afsk_modem_channel_t *out)
{
    *out = s_chan;
}

void afsk_modem_set_channel(const afsk_modem_channel_t *in)
{
    afsk_modem_channel_t c = *in;
    c.txdelay_ms = c.txdelay_ms < 0 ? 0 : c.txdelay_ms > 1000 ? 1000 : c.txdelay_ms; /* fits s_tx_bits */
    c.persist = c.persist < 0 ? 0 : c.persist > 255 ? 255 : c.persist;
    c.slot_ms = c.slot_ms < 0 ? 0 : c.slot_ms > 2550 ? 2550 : c.slot_ms;
    c.txtail_flags = c.txtail_flags < 1 ? 1 : c.txtail_flags > 64 ? 64 : c.txtail_flags;
    s_chan = c; /* read by the TX task only between transmissions */
}
