#pragma once
/*
 * TCP debug server for the AFSK modem, one client per port.
 *
 * Console, works with nc or telnet, type "help" once connected:
 *     nc <ip> 2323
 * Audio tap, a WAV of the RX audio. Live until you disconnect, or "tap 12k 3" on the
 * console first to record 3 s into RAM and then send it (better on a slow link):
 *     nc <ip> 2324 > capture.wav
 * Replay it on the PC with: test/test_modem --wav capture.wav [--trace out.csv]
 */
#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t debug_server_start(uint16_t console_port, uint16_t audio_port);

#ifdef __cplusplus
}
#endif
