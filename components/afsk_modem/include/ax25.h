#pragma once
/*
 * AX.25 UI frame helpers: CRC, TNC2 text <-> raw frame.
 * Portable C, no ESP-IDF dependencies (also built by the host test).
 */
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define AX25_MAX_REPEATERS 8
/* 10 addresses * 7 + control + PID + 256 info + 2 FCS */
#define AX25_MAX_FRAME     (10 * 7 + 2 + 256 + 2)
#define AX25_MIN_FRAME     (2 * 7 + 1 + 2)

/* CRC-16/X.25 as used for the AX.25 FCS (sent low byte first). */
uint16_t ax25_crc(const uint8_t *data, int len);

/*
 * Build a UI frame (control 0x03, PID 0xF0) from TNC2 text, e.g.
 *   "N0CALL-9>TEST,WIDE1-1,WIDE2-1:hello there"
 * Writes the frame including FCS into out. Returns its length, or -1 on parse error / overflow.
 */
int ax25_from_tnc2(const char *tnc2, uint8_t *out, int out_size);

/*
 * Format a received frame (without FCS) as TNC2 text. Non-printable info bytes are shown as <0xNN>.
 * Returns the string length, or -1 if the address field is malformed.
 */
int ax25_to_tnc2(const uint8_t *frame, int len, char *out, int out_size);

#ifdef __cplusplus
}
#endif
