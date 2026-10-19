#include <stdio.h>

#include <esp_log.h>
#include <esp_err.h>
#include <nvs_flash.h>
#include <esp_ota_ops.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "debug_server.h"
#include "afsk_modem.h"
#include "kiss_server.h"

const static char TAG[] = "main.c";

/*
 * Internal ADC / DAC wiring:
 *   RX audio -> GPIO36 (ADC1_CH0). Bias it to ~1.6 V (e.g. 2 x 10k divider) and AC-couple the
 *               audio in through ~1 uF. With 12 dB attenuation the ADC spans about 0.15..3.1 V.
 *   TX audio <- GPIO25 (DAC1), idles at mid-scale (~1.65 V). AC-couple out and divide down for a
 *               mic input; an RC low-pass (~5 kHz) smooths the 40 kHz DAC steps.
 *   PTT      <- GPIO33, active low.
 *
 * Loopback test: a plain wire GPIO25 -> GPIO36 (the DAC already idles at mid-scale), and every
 * beacon below should be decoded back as "RX ...". Web GUI on http://<ip>/.
 */
#define BOARD_PTT_GPIO      33
#define DEBUG_CONSOLE_PORT  2323 /* nc <ip> 2323, then "help" */
#define DEBUG_AUDIO_PORT    2324 /* nc <ip> 2324 > capture.wav */
#define KISS_PORT           8001 /* KISS over TCP for Linux kissattach (see kiss_server.h) */
#define BEACON_INTERVAL_MS  30000 /* 0 = no test beacon */
#define BEACON_TEXT         "N0CALL-1>BEACON:hello #%d" /* AX.25 UI, so Linux accepts it too */

static void modem_rx(const uint8_t *frame, int len, const char *ax25, void *ctx)
{
    kiss_server_frame_received(frame, len);
    if (ax25)
        ESP_LOGI(TAG, "RX AX.25 %s", ax25);
    else
        ESP_LOGI(TAG, "RX %d bytes: %.*s", len, len, (const char *)frame);
}

/* Test frame every BEACON_INTERVAL_MS; with the loopback cable each one should come back as RX. */
static void beacon_task(void *arg)
{
    char text[64];
    for (int n = 1;; n++)
    {
        vTaskDelay(pdMS_TO_TICKS(BEACON_INTERVAL_MS));
        snprintf(text, sizeof(text), BEACON_TEXT, n);
        esp_err_t err = afsk_modem_send(text);
        ESP_LOGI(TAG, "TX %s (%s)", text, esp_err_to_name(err));
    }
}

esp_err_t nvs_init()
{
    esp_err_t ret = nvs_flash_init();

    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    return ret;
}

extern void wifi_init_sta(void);
extern esp_err_t http_server_init(void);
void app_main(void)
{
    ESP_LOGI(TAG, "Hi from main. Concha la pelota " __DATE__ " " __TIME__);
    ESP_ERROR_CHECK(nvs_init());

    wifi_init_sta();
    ESP_ERROR_CHECK(http_server_init());

    afsk_modem_config_t modem_cfg = AFSK_MODEM_DEFAULT_CONFIG();
    modem_cfg.ptt_gpio = BOARD_PTT_GPIO;
    modem_cfg.rx_cb = modem_rx;
    ESP_ERROR_CHECK(afsk_modem_start(&modem_cfg));
    ESP_ERROR_CHECK(debug_server_start(DEBUG_CONSOLE_PORT, DEBUG_AUDIO_PORT));

    ESP_ERROR_CHECK(kiss_server_start(KISS_PORT));

    if (BEACON_INTERVAL_MS > 0)
        xTaskCreate(beacon_task, "beacon", 3072, NULL, 5, NULL);

    /* Mark current app as valid */
    const esp_partition_t *partition = esp_ota_get_running_partition();
    printf("Currently running partition: %s\r\n", partition->label);

    esp_ota_img_states_t ota_state;
    if (esp_ota_get_state_partition(partition, &ota_state) == ESP_OK)
    {
        if (ota_state == ESP_OTA_IMG_PENDING_VERIFY)
        {
            esp_ota_mark_app_valid_cancel_rollback();
        }
    }
}
