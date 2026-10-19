#pragma once
/*
 * Bell 202 AFSK modulator.
 *
 * The encoder turns frames (FCS included) into a packed array of tone bits, with HDLC
 * flags, bit stuffing and NRZI already applied (1 = mark, 0 = space).
 * afsk_mod_next() then makes one 8-bit DAC sample at a time. It's integer only and
 * inline because it runs inside a timer ISR, and the FPU is off limits there on Xtensa.
 */
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define AFSK_BAUD     1200
#define AFSK_MARK_HZ  1200
#define AFSK_SPACE_HZ 2200

typedef struct
{
    const uint8_t *bits;
    uint32_t nbits;
    uint32_t idx;
    uint32_t loop_samples; /* != 0: repeat bits[] until this many samples were produced */

    uint32_t bitclk;     /* wraps once per bit */
    uint32_t bitclk_inc;

    uint32_t phase;      /* NCO, continuous across tone changes */
    uint32_t inc_mark;
    uint32_t inc_space;

    uint8_t lut[256];    /* 128 +/- level * sin() */
} afsk_mod_t;

/*
 * Incremental encoder, for sending several frames in one transmission:
 *   afsk_enc_init(); afsk_enc_flags(preamble); afsk_enc_frame(); afsk_enc_flags(1); afsk_enc_frame(); ...
 *   afsk_enc_flags(tail); then e.nbits tone bits are in bits[].
 * Each call returns 0, or -1 if the buffer is full (the encoder is then left as it was before the call).
 */
typedef struct
{
    uint8_t *bits;
    int size_bits;
    int nbits;
    int tone; /* current NRZI level, 1 = mark */
} afsk_encoder_t;

void afsk_enc_init(afsk_encoder_t *e, uint8_t *bits, int bits_size);
int afsk_enc_flags(afsk_encoder_t *e, int count);
int afsk_enc_frame(afsk_encoder_t *e, const uint8_t *frame, int len); /* frame includes FCS */

/*
 * One frame with preamble and tail. Returns the number of tone bits written into bits[]
 * (packed LSB first), or -1 if bits_size bytes were not enough.
 */
int afsk_encode_frame(const uint8_t *frame, int len, int preamble_flags, int tail_flags,
                      uint8_t *bits, int bits_size);

/* level: peak deviation around the 128 midpoint, 1..127 */
void afsk_mod_init(afsk_mod_t *m, int sample_rate, int level);
void afsk_mod_load(afsk_mod_t *m, const uint8_t *bits, uint32_t nbits);
/* Repeat a raw tone-bit pattern (1 = mark) for `samples` samples, e.g. steady tones for level setting. */
void afsk_mod_load_loop(afsk_mod_t *m, const uint8_t *bits, uint32_t nbits, uint32_t samples);

/* Writes the next sample to *out. Returns 0 once all bits have been sent. */
#if defined(__GNUC__)
__attribute__((always_inline))
#endif
static inline int afsk_mod_next(afsk_mod_t *m, uint8_t *out)
{
    if (m->loop_samples)
    {
        if (m->idx >= m->nbits)
            m->idx = 0;
        if (--m->loop_samples == 0)
            m->nbits = 0; /* this is the last sample; the next call returns 0 */
    }
    else if (m->idx >= m->nbits)
    {
        return 0;
    }

    int mark = (m->bits[m->idx >> 3] >> (m->idx & 7)) & 1;
    m->phase += mark ? m->inc_mark : m->inc_space;
    *out = m->lut[m->phase >> 24];

    uint32_t prev = m->bitclk;
    m->bitclk += m->bitclk_inc;
    if (m->bitclk < prev)
        m->idx++;
    return 1;
}

#ifdef __cplusplus
}
#endif
