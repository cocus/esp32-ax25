#pragma once
/*
 * KISS over TCP (the Dire Wolf convention, usually port 8001), so Linux can use the modem as an
 * AX.25 interface:
 *     socat pty,link=/tmp/kiss,raw,echo=0 tcp:<ip>:8001 &
 *     kissattach /tmp/kiss radio 44.0.0.1
 *
 * Data frames from clients are sent on air (FCS added); frames received on air are sent to all
 * clients. KISS parameter commands (TXDELAY, P, SlotTime, TXtail, FullDuplex) set the modem's
 * channel access.
 */
#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t kiss_server_start(uint16_t port);

/* Call from the modem's RX callback with each good frame (FCS stripped). Never blocks. */
void kiss_server_frame_received(const uint8_t *frame, int len);

typedef struct
{
    int clients;
    uint32_t to_air;      /* data frames accepted from clients */
    uint32_t from_air;    /* frames forwarded to clients */
    uint32_t dropped;     /* received frames not forwarded (clients too slow) */
    uint32_t bad_kiss;    /* malformed or oversized KISS frames from clients */
} kiss_server_stats_t;

void kiss_server_get_stats(kiss_server_stats_t *out);

#ifdef __cplusplus
}
#endif
