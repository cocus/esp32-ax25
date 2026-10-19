#include "afsk_demod.h"

#include <math.h>
#include <string.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* AGC constants from Dire Wolf (tuned for 44.1 kHz), decay rescaled for 12 kHz. */
#define AGC_FAST_ATTACK 0.70f
#define AGC_SLOW_DECAY  (0.000090f * 44100.0f / AFSK_DEMOD_FS)

/* PLL inertia on a data transition: closer to 1 = smaller correction. */
#define PLL_LOCKED_INERTIA    0.74f
#define PLL_SEARCHING_INERTIA 0.50f

static float cos_tab[256];

/* Ideal lowpass impulse response at offset x (in samples), cutoff fc. */
static double sinc_lp(double fc, double fs, double x)
{
    if (x == 0.0)
        return 2.0 * fc / fs;
    return sin(2.0 * M_PI * fc / fs * x) / (M_PI * x);
}

static double hann(int n, int taps)
{
    return 0.5 - 0.5 * cos(2.0 * M_PI * (n + 1) / (taps + 1));
}

static void design_lowpass(float *h, int taps, double fc, double fs)
{
    double sum = 0.0;
    for (int n = 0; n < taps; n++)
    {
        double x = n - (taps - 1) / 2.0;
        h[n] = (float)(sinc_lp(fc, fs, x) * hann(n, taps));
        sum += h[n];
    }
    for (int n = 0; n < taps; n++)
        h[n] = (float)(h[n] / sum); /* unity gain at DC */
}

static void design_bandpass(float *h, int taps, double f1, double f2, double fs)
{
    for (int n = 0; n < taps; n++)
    {
        double x = n - (taps - 1) / 2.0;
        h[n] = (float)((sinc_lp(f2, fs, x) - sinc_lp(f1, fs, x)) * hann(n, taps));
    }
    /* unity gain at the centre frequency */
    double f0 = 0.5 * (f1 + f2), re = 0.0, im = 0.0;
    for (int n = 0; n < taps; n++)
    {
        re += h[n] * cos(2.0 * M_PI * f0 / fs * n);
        im -= h[n] * sin(2.0 * M_PI * f0 / fs * n);
    }
    double mag = sqrt(re * re + im * im);
    for (int n = 0; n < taps; n++)
        h[n] = (float)(h[n] / mag);
}

/* Ring buffer stored twice so the newest `taps` samples are always contiguous at buf[pos]. */
static inline void ring_push(float *buf, int *pos, int taps, float x)
{
    buf[*pos] = x;
    buf[*pos + taps] = x;
    if (++*pos == taps)
        *pos = 0;
}

static inline float ring_fir(const float *buf, int pos, const float *h, int taps)
{
    const float *x = buf + pos;
    float acc = 0.0f;
    for (int k = 0; k < taps; k++)
        acc += h[k] * x[k];
    return acc;
}

static inline float ring_sum(const float *buf, int pos, int taps)
{
    const float *x = buf + pos;
    float acc = 0.0f;
    for (int k = 0; k < taps; k++)
        acc += x[k];
    return acc;
}

static inline float agc(float in, float *peak, float *valley)
{
    *peak += (in >= *peak ? AGC_FAST_ATTACK : AGC_SLOW_DECAY) * (in - *peak);
    *valley += (in <= *valley ? AGC_FAST_ATTACK : AGC_SLOW_DECAY) * (in - *valley);
    if (*peak <= *valley)
        return 0.0f;
    return (in - 0.5f * (*peak + *valley)) / (*peak - *valley);
}

void afsk_demod_init(afsk_demod_t *d, afsk_frame_cb_t cb, void *ctx)
{
    memset(d, 0, sizeof(*d));
    d->cb = cb;
    d->cb_ctx = ctx;

    if (cos_tab[0] == 0.0f)
        for (int i = 0; i < 256; i++)
            cos_tab[i] = (float)cos(2.0 * M_PI * i / 256.0);

    design_lowpass(d->dec_h, AFSK_DEMOD_DEC_TAPS, 4000.0, AFSK_DEMOD_FS_IN);
    design_bandpass(d->bpf_h, AFSK_DEMOD_BPF_TAPS, 900.0, 2500.0, AFSK_DEMOD_FS);

    d->m_inc = (uint32_t)llround(1200.0 * 4294967296.0 / AFSK_DEMOD_FS);
    d->s_inc = (uint32_t)llround(2200.0 * 4294967296.0 / AFSK_DEMOD_FS);
}

static void hdlc_bit(afsk_demod_t *d, int bit)
{
    d->pattern = (uint8_t)((d->pattern << 1) | bit);

    if (d->pattern == 0x7E)
    {
        d->flags_seen++;
        /* Closing flag: the 7 flag bits before this one were shifted in as data. */
        if (d->in_frame && d->nbits == 7 && d->flen >= AFSK_DEMOD_MIN_FRAME)
        {
            uint16_t fcs = (uint16_t)(d->frame[d->flen - 2] | (d->frame[d->flen - 1] << 8));
            afsk_frame_info_t info = {
                .crc_ok = ax25_crc(d->frame, d->flen - 2) == fcs,
                .mark_level = afsk_demod_mark_level(d),
                .space_level = afsk_demod_space_level(d),
            };
            if (info.crc_ok)
                d->frames_ok++;
            else
                d->frames_bad_crc++;
            if (d->cb)
                d->cb(d->frame, info.crc_ok ? d->flen - 2 : d->flen, &info, d->cb_ctx);
        }
        d->in_frame = true;
        d->flen = 0;
        d->nbits = 0;
        d->cur_byte = 0;
        return;
    }

    if ((d->pattern & 0x7F) == 0x7F)
    {
        /* 7 ones in a row: abort / idle */
        d->in_frame = false;
        return;
    }

    if (!d->in_frame)
        return;

    if ((d->pattern & 0x3F) == 0x3E)
        return; /* stuffed zero after five ones */

    d->cur_byte = (uint8_t)((d->cur_byte >> 1) | (bit << 7));
    if (++d->nbits == 8)
    {
        if (d->flen >= AX25_MAX_FRAME)
        {
            d->in_frame = false;
            return;
        }
        d->frame[d->flen++] = d->cur_byte;
        d->nbits = 0;
    }
}

static void process_12k(afsk_demod_t *d, float x)
{
    ring_push(d->bpf_buf, &d->bpf_pos, AFSK_DEMOD_BPF_TAPS, x);
    float y = ring_fir(d->bpf_buf, d->bpf_pos, d->bpf_h, AFSK_DEMOD_BPF_TAPS);

    /* Mix down each tone to DC and integrate over one symbol (matched filter). */
    int cp = d->corr_pos;
    ring_push(d->corr[0], &cp, AFSK_DEMOD_SPB, y * cos_tab[d->m_phase >> 24]);
    cp = d->corr_pos;
    ring_push(d->corr[1], &cp, AFSK_DEMOD_SPB, y * cos_tab[(uint8_t)((d->m_phase >> 24) - 64)]);
    cp = d->corr_pos;
    ring_push(d->corr[2], &cp, AFSK_DEMOD_SPB, y * cos_tab[d->s_phase >> 24]);
    cp = d->corr_pos;
    ring_push(d->corr[3], &cp, AFSK_DEMOD_SPB, y * cos_tab[(uint8_t)((d->s_phase >> 24) - 64)]);
    d->corr_pos = cp;
    d->m_phase += d->m_inc;
    d->s_phase += d->s_inc;

    float mi = ring_sum(d->corr[0], cp, AFSK_DEMOD_SPB);
    float mq = ring_sum(d->corr[1], cp, AFSK_DEMOD_SPB);
    float si = ring_sum(d->corr[2], cp, AFSK_DEMOD_SPB);
    float sq = ring_sum(d->corr[3], cp, AFSK_DEMOD_SPB);
    float m_amp = sqrtf(mi * mi + mq * mq);
    float s_amp = sqrtf(si * si + sq * sq);

    float demod = agc(m_amp, &d->m_peak, &d->m_valley) - agc(s_amp, &d->s_peak, &d->s_valley);
    int demod_bit = demod > 0.0f;

    /* Bit clock: sample when the accumulator wraps from positive to negative. */
    int32_t prev = d->pll;
    d->pll = (int32_t)((uint32_t)d->pll + (uint32_t)(4294967296.0 / AFSK_DEMOD_SPB));
    int sampled = 0;
    if (prev > 0 && d->pll < 0)
    {
        sampled = 1;
        hdlc_bit(d, demod_bit == d->prev_raw_bit); /* NRZI: no change = 1 */
        d->prev_raw_bit = demod_bit;
    }

    if (demod_bit != d->prev_demod_bit)
    {
        /*
         * Carrier detect: with a real signal, transitions arrive within about a sample of where the
         * clock expects them (pll == 0); noise transitions land anywhere (~25% inside this window).
         */
        int on_time = d->pll > -(1 << 29) && d->pll < (1 << 29);
        d->dcd_history = (uint16_t)((d->dcd_history << 1) | on_time);
        int good = __builtin_popcount(d->dcd_history);
        if (good >= 12)
            d->dcd = true;
        else if (good <= 6)
            d->dcd = false;

        /* Pull the clock so transitions sit at pll == 0, i.e. mid-way between bit samples. */
        d->pll = (int32_t)(d->pll * ((d->dcd || d->in_frame) ? PLL_LOCKED_INERTIA : PLL_SEARCHING_INERTIA));
    }
    d->prev_demod_bit = demod_bit;

    if (d->trace)
    {
        d->trace->in = x;
        d->trace->bpf = y;
        d->trace->mark = m_amp;
        d->trace->space = s_amp;
        d->trace->demod = demod;
        d->trace->pll = d->pll;
        d->trace->sampled = sampled;
        d->trace->dcd = d->dcd;
    }
}

bool afsk_demod_process(afsk_demod_t *d, float x)
{
    d->dc += (x - d->dc) * (1.0f / 2048.0f);
    ring_push(d->dec_buf, &d->dec_pos, AFSK_DEMOD_DEC_TAPS, x - d->dc);

    if (++d->dec_phase < AFSK_DEMOD_DECIM)
        return false;
    d->dec_phase = 0;

    process_12k(d, ring_fir(d->dec_buf, d->dec_pos, d->dec_h, AFSK_DEMOD_DEC_TAPS));
    return true;
}

void afsk_demod_process_decimated(afsk_demod_t *d, float x)
{
    process_12k(d, x);
}

/* A tone of amplitude A (after the bandpass) gives a correlator magnitude of A * SPB / 2. */
float afsk_demod_mark_level(const afsk_demod_t *d)
{
    return d->m_peak * (2.0f / AFSK_DEMOD_SPB);
}

float afsk_demod_space_level(const afsk_demod_t *d)
{
    return d->s_peak * (2.0f / AFSK_DEMOD_SPB);
}
