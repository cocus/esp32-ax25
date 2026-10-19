#include "afsk_mod.h"

#include <math.h>
#include <string.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

static int put_raw(afsk_encoder_t *e, int bit)
{
    if (e->nbits >= e->size_bits)
        return -1;
    if (!bit)
        e->tone ^= 1; /* NRZI: 0 = change tone, 1 = keep */
    if (e->tone)
        e->bits[e->nbits >> 3] |= (uint8_t)(1 << (e->nbits & 7));
    else
        e->bits[e->nbits >> 3] &= (uint8_t)~(1 << (e->nbits & 7));
    e->nbits++;
    return 0;
}

void afsk_enc_init(afsk_encoder_t *e, uint8_t *bits, int bits_size)
{
    e->bits = bits;
    e->size_bits = bits_size * 8;
    e->nbits = 0;
    e->tone = 1;
}

int afsk_enc_flags(afsk_encoder_t *e, int count)
{
    afsk_encoder_t saved = *e;
    for (int i = 0; i < count; i++)
        for (int b = 0; b < 8; b++)
            if (put_raw(e, (0x7E >> b) & 1) < 0)
            {
                *e = saved;
                return -1;
            }
    return 0;
}

int afsk_enc_frame(afsk_encoder_t *e, const uint8_t *frame, int len)
{
    afsk_encoder_t saved = *e;
    int ones = 0;
    for (int i = 0; i < len; i++)
    {
        for (int b = 0; b < 8; b++)
        {
            int bit = (frame[i] >> b) & 1;
            if (put_raw(e, bit) < 0)
                goto full;
            if (bit && ++ones == 5)
            {
                if (put_raw(e, 0) < 0) /* bit stuffing */
                    goto full;
                ones = 0;
            }
            else if (!bit)
            {
                ones = 0;
            }
        }
    }
    return 0;

full:
    *e = saved;
    return -1;
}

int afsk_encode_frame(const uint8_t *frame, int len, int preamble_flags, int tail_flags,
                      uint8_t *bits, int bits_size)
{
    afsk_encoder_t e;
    afsk_enc_init(&e, bits, bits_size);
    if (afsk_enc_flags(&e, preamble_flags) < 0 || afsk_enc_frame(&e, frame, len) < 0 ||
        afsk_enc_flags(&e, tail_flags) < 0)
        return -1;
    return e.nbits;
}

void afsk_mod_init(afsk_mod_t *m, int sample_rate, int level)
{
    memset(m, 0, sizeof(*m));
    if (level < 1)
        level = 1;
    if (level > 127)
        level = 127;
    for (int i = 0; i < 256; i++)
        m->lut[i] = (uint8_t)lround(128.0 + level * sin(2.0 * M_PI * i / 256.0));

    m->inc_mark = (uint32_t)llround((double)AFSK_MARK_HZ * 4294967296.0 / sample_rate);
    m->inc_space = (uint32_t)llround((double)AFSK_SPACE_HZ * 4294967296.0 / sample_rate);
    m->bitclk_inc = (uint32_t)llround((double)AFSK_BAUD * 4294967296.0 / sample_rate);
}

void afsk_mod_load(afsk_mod_t *m, const uint8_t *bits, uint32_t nbits)
{
    m->bits = bits;
    m->nbits = nbits;
    m->idx = 0;
    m->bitclk = 0;
    m->loop_samples = 0;
}

void afsk_mod_load_loop(afsk_mod_t *m, const uint8_t *bits, uint32_t nbits, uint32_t samples)
{
    afsk_mod_load(m, bits, nbits);
    m->loop_samples = samples;
}
