#pragma once
/*
 * Bell 202 AFSK 1200 demodulator + HDLC receiver.
 *
 *   48 kHz in -> DC removal -> anti-alias FIR, decimate by 4 -> 12 kHz
 *   -> bandpass 900..2500 Hz
 *   -> mark / space quadrature correlators (NCO mix + 1-symbol moving sum)
 *   -> per-tone AGC (peak/valley, as in Dire Wolf), demod = mark - space
 *   -> Dire Wolf style bit-clock PLL -> NRZI -> HDLC (unstuff, flags, CRC)
 *
 * Portable C (float), no ESP-IDF dependencies: the host test builds it as-is.
 */
#include <stdbool.h>
#include <stdint.h>

#include "ax25.h"

#ifdef __cplusplus
extern "C" {
#endif

#define AFSK_DEMOD_FS_IN  48000
#define AFSK_DEMOD_DECIM  4
#define AFSK_DEMOD_FS     (AFSK_DEMOD_FS_IN / AFSK_DEMOD_DECIM) /* 12 kHz */
#define AFSK_DEMOD_SPB    (AFSK_DEMOD_FS / 1200)                /* 10 samples per bit */

#define AFSK_DEMOD_DEC_TAPS 48
#define AFSK_DEMOD_BPF_TAPS 25

/* Shortest candidate passed on: 1 payload byte + 2 FCS. Frames are not required to be AX.25. */
#define AFSK_DEMOD_MIN_FRAME 3

typedef struct
{
    bool crc_ok;
    float mark_level;  /* tone amplitudes at the end of the frame, as a fraction of input full scale */
    float space_level;
} afsk_frame_info_t;

/*
 * Called for every flag-delimited candidate of plausible length. crc_ok frames have the FCS
 * stripped from len; bad ones are passed whole (useful for debugging, ignore otherwise).
 */
typedef void (*afsk_frame_cb_t)(const uint8_t *frame, int len, const afsk_frame_info_t *info, void *ctx);

/* Optional per-sample (12 kHz) snapshot for plotting. */
typedef struct
{
    float in;        /* decimated input */
    float bpf;       /* after bandpass */
    float mark;      /* mark tone amplitude */
    float space;     /* space tone amplitude */
    float demod;     /* AGC'd mark - space, > 0 means mark */
    int32_t pll;     /* bit clock; a bit is sampled when it wraps positive -> negative */
    int sampled;     /* 1 on the samples where a bit was taken */
    int dcd;         /* carrier detect */
} afsk_demod_trace_t;

typedef struct
{
    /* input conditioning */
    float dc;
    float dec_h[AFSK_DEMOD_DEC_TAPS];
    float dec_buf[2 * AFSK_DEMOD_DEC_TAPS];
    int dec_pos;
    int dec_phase;

    float bpf_h[AFSK_DEMOD_BPF_TAPS];
    float bpf_buf[2 * AFSK_DEMOD_BPF_TAPS];
    int bpf_pos;

    /* correlators */
    uint32_t m_phase, m_inc;
    uint32_t s_phase, s_inc;
    float corr[4][2 * AFSK_DEMOD_SPB]; /* mark I, mark Q, space I, space Q */
    int corr_pos;

    /* AGC */
    float m_peak, m_valley;
    float s_peak, s_valley;

    /* bit clock */
    int32_t pll;
    int prev_demod_bit;

    /* carrier detect: did the last 16 transitions land where the bit clock expects them? */
    uint16_t dcd_history;
    bool dcd;

    /* NRZI + HDLC */
    int prev_raw_bit;
    uint8_t pattern;
    uint8_t cur_byte;
    int nbits;
    bool in_frame;
    int flen;
    uint8_t frame[AX25_MAX_FRAME];

    afsk_frame_cb_t cb;
    void *cb_ctx;
    afsk_demod_trace_t *trace;

    /* statistics */
    uint32_t frames_ok;
    uint32_t frames_bad_crc;
    uint32_t flags_seen;  /* HDLC flags: ~150/s during a preamble, ~5/s on random noise */
} afsk_demod_t;

void afsk_demod_init(afsk_demod_t *d, afsk_frame_cb_t cb, void *ctx);

/*
 * x: one 48 kHz sample, any DC offset, roughly within +/-1 full scale.
 * Returns true when it also produced a 12 kHz sample (d->trace, if set, was updated).
 */
bool afsk_demod_process(afsk_demod_t *d, float x);

/* Feed an already decimated 12 kHz, DC-free sample (e.g. replaying a 12 kHz capture). */
void afsk_demod_process_decimated(afsk_demod_t *d, float x);

/*
 * Current tone amplitudes (AGC peaks) as a fraction of input full scale, measured after the
 * bandpass, so roughly 20-30% below the tone amplitude at the ADC. Their ratio shows the twist.
 */
float afsk_demod_mark_level(const afsk_demod_t *d);
float afsk_demod_space_level(const afsk_demod_t *d);

#ifdef __cplusplus
}
#endif
