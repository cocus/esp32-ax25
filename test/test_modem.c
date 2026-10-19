/*
 * Host test for components/afsk_modem: TX encoder + modulator -> simulated
 * analog path (resampling, clock offset, twist, noise, 12-bit ADC) -> demod.
 *
 * Build (from repo root), any of:
 *   clang -O2 -Icomponents/afsk_modem/include test/test_modem.c components/afsk_modem/src/afsk_demod.c components/afsk_modem/src/afsk_mod.c components/afsk_modem/src/ax25.c -o test/test_modem.exe
 *   gcc   (same arguments, add -lm)
 * Run:
 *   test/test_modem                          run all scenarios
 *   test/test_modem --trace                  also write afsk_demod_trace.csv for the first clean frame
 *   test/test_modem --wav cap.wav [--trace t.csv]
 *                                                 decode a capture from the ESP32 audio tap (48 kHz or 12 kHz,
 *                                                 16-bit) and optionally dump the per-sample demod trace
 *   test/test_modem --make-wav out.wav       write the "SNR ~12 dB" scenario as a 48 kHz tap-style WAV
 */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "afsk_demod.h"
#include "afsk_mod.h"
#include "ax25.h"

#define TX_FS 40000 /* matches the ESP32 DAC timer rate */

static const char *messages[] = {
    "N0CALL-9>APRS,WIDE1-1,WIDE2-1:!3436.00S/05822.00W>esp32 afsk modem test",
    "N0CALL>APZ001:>status text with ~ and | and 11111111 ones",
    "LU1ABC-7>APRS,LU2XYZ-1*,WIDE2-1::N0CALL   :hello there{001",
    "N0CALL-1>BEACON:`(_fn\"Oj/]\"4-}=",
};
#define NMSG (int)(sizeof(messages) / sizeof(messages[0]))

typedef struct
{
    double adc_ppm;      /* ADC clock error vs TX clock */
    double amplitude;    /* peak, in ADC counts */
    double noise_rms;    /* in ADC counts */
    double twist_db;     /* + = space louder than mark (pre-emphasis), - = de-emphasis */
    int adc_offset;      /* DC bias in counts */
} scenario_t;

static int decoded_count;
static int decoded_match[NMSG];
static int print_frames;       /* replay mode: print every candidate */
static double replay_time;     /* seconds into the replayed file */
static FILE *wav_out;          /* --make-wav */
static double dcd_on_signal, dcd_on_idle; /* % of time carrier detect was on, last run */

static void on_frame(const uint8_t *frame, int len, const afsk_frame_info_t *info, void *ctx)
{
    (void)ctx;
    char text[AX25_MAX_FRAME * 6];
    int ok = ax25_to_tnc2(frame, info->crc_ok ? len : len - 2, text, sizeof(text)) >= 0;

    if (print_frames)
    {
        double twist = 20.0 * log10(info->space_level / info->mark_level);
        if (info->crc_ok && ok)
            printf("%9.3fs  RX  mark %.3f space %.3f (twist %+.1f dB): %s\n", replay_time, info->mark_level,
                   info->space_level, twist, text);
        else
        {
            printf("%9.3fs  BAD %d bytes, mark %.3f space %.3f: ", replay_time, len, info->mark_level, info->space_level);
            for (int i = 0; i < len; i++)
                printf("%02X", frame[i]);
            printf(ok ? " | %s\n" : "\n", text);
        }
    }

    if (!info->crc_ok || !ok)
        return;
    decoded_count++;
    for (int i = 0; i < NMSG; i++)
        if (strcmp(text, messages[i]) == 0)
            decoded_match[i]++;
}

static void put_le(uint8_t *p, uint32_t v, int bytes)
{
    for (int i = 0; i < bytes; i++)
        p[i] = (uint8_t)(v >> (8 * i));
}

static void wav_write_header(FILE *f, uint32_t rate, uint32_t data_bytes)
{
    uint8_t h[44];
    memcpy(h, "RIFF", 4);
    put_le(h + 4, 36 + data_bytes, 4);
    memcpy(h + 8, "WAVEfmt ", 8);
    put_le(h + 16, 16, 4);
    put_le(h + 20, 1, 2); /* PCM */
    put_le(h + 22, 1, 2); /* mono */
    put_le(h + 24, rate, 4);
    put_le(h + 28, rate * 2, 4);
    put_le(h + 32, 2, 2);
    put_le(h + 34, 16, 2);
    memcpy(h + 36, "data", 4);
    put_le(h + 40, data_bytes, 4);
    fseek(f, 0, SEEK_SET);
    fwrite(h, 1, sizeof(h), f);
}

static uint32_t get_le(const uint8_t *p, int bytes)
{
    uint32_t v = 0;
    for (int i = bytes - 1; i >= 0; i--)
        v = (v << 8) | p[i];
    return v;
}

/* Decode a 16-bit PCM WAV. The ESP32 tap declares a bogus huge length, so read until EOF. */
static int replay_wav(const char *path, const char *trace_path)
{
    FILE *f = fopen(path, "rb");
    if (!f)
    {
        printf("cannot open %s\n", path);
        return 2;
    }
    uint8_t h[12], ch[8];
    if (fread(h, 1, 12, f) != 12 || memcmp(h, "RIFF", 4) || memcmp(h + 8, "WAVE", 4))
    {
        printf("%s: not a WAV file\n", path);
        return 2;
    }

    uint32_t rate = 0, channels = 0, bits = 0;
    for (;;)
    {
        if (fread(ch, 1, 8, f) != 8)
        {
            printf("%s: no data chunk\n", path);
            return 2;
        }
        uint32_t size = get_le(ch + 4, 4);
        if (!memcmp(ch, "fmt ", 4))
        {
            uint8_t fmt[16];
            if (size < 16 || fread(fmt, 1, 16, f) != 16)
                return 2;
            channels = get_le(fmt + 2, 2);
            rate = get_le(fmt + 4, 4);
            bits = get_le(fmt + 14, 2);
            fseek(f, size - 16 + (size & 1), SEEK_CUR);
        }
        else if (!memcmp(ch, "data", 4))
        {
            break;
        }
        else
        {
            fseek(f, size + (size & 1), SEEK_CUR);
        }
    }
    if (bits != 16 || channels < 1 || (rate != AFSK_DEMOD_FS_IN && rate != AFSK_DEMOD_FS))
    {
        printf("%s: need 16-bit PCM at %d or %d Hz (got %u bit, %u ch, %u Hz)\n", path, AFSK_DEMOD_FS_IN,
               AFSK_DEMOD_FS, bits, channels, rate);
        return 2;
    }

    static afsk_demod_t d;
    afsk_demod_trace_t tr;
    afsk_demod_init(&d, on_frame, NULL);
    print_frames = 1;

    FILE *tf = NULL;
    if (trace_path)
    {
        tf = fopen(trace_path, "w");
        fprintf(tf, "t,in,bpf,mark,space,demod,pll,sampled\n");
        d.trace = &tr;
    }

    printf("%s: %u Hz, %u channel(s)\n", path, rate, channels);
    int16_t frame_buf[8];
    long n = 0, n12 = 0;
    int32_t vmin = 32767, vmax = -32768;
    while (fread(frame_buf, 2, channels, f) == channels)
    {
        int16_t s = frame_buf[0];
        if (s < vmin)
            vmin = s;
        if (s > vmax)
            vmax = s;
        replay_time = (double)n++ / rate;
        int out12k = 1;
        if (rate == AFSK_DEMOD_FS_IN)
            out12k = afsk_demod_process(&d, s / 32768.0f);
        else
            afsk_demod_process_decimated(&d, s / 32768.0f);
        if (tf && out12k)
            fprintf(tf, "%.6f,%g,%g,%g,%g,%g,%ld,%d\n", (double)n12++ / AFSK_DEMOD_FS, tr.in, tr.bpf, tr.mark,
                    tr.space, tr.demod, (long)tr.pll, tr.sampled);
    }
    fclose(f);
    if (tf)
        fclose(tf);

    printf("%.1f s of audio, sample range %d..%d", (double)n / rate, vmin, vmax);
    if (rate == AFSK_DEMOD_FS_IN)
        printf(" (12-bit ADC codes %d..%d)", vmin / 16 + 2048, vmax / 16 + 2048);
    printf("\n");
    printf("frames ok %u, bad CRC %u, HDLC flags %u\n", (unsigned)d.frames_ok, (unsigned)d.frames_bad_crc,
           (unsigned)d.flags_seen);
    return 0;
}

static double gauss(void)
{
    double u1 = (rand() + 1.0) / (RAND_MAX + 2.0), u2 = (rand() + 1.0) / (RAND_MAX + 2.0);
    return sqrt(-2.0 * log(u1)) * cos(2.0 * 3.14159265358979 * u2);
}

/* Build the whole TX audio (DAC codes, centred on 0) for all messages with silence gaps. */
static float *build_tx(int *out_len)
{
    static uint8_t bits[2048];
    static afsk_mod_t mod;
    int cap = TX_FS * 30, n = 0;
    float *buf = calloc(cap, sizeof(float));

    afsk_mod_init(&mod, TX_FS, 100);
    for (int m = 0; m < NMSG; m++)
    {
        uint8_t frame[AX25_MAX_FRAME];
        int flen = ax25_from_tnc2(messages[m], frame, sizeof(frame));
        if (flen < 0)
        {
            printf("encode failed: %s\n", messages[m]);
            exit(2);
        }
        int nb = afsk_encode_frame(frame, flen, 45, 3, bits, sizeof(bits));
        afsk_mod_load(&mod, bits, nb);

        for (int i = 0; i < TX_FS / 4; i++) /* 250 ms silence */
            buf[n++] = 0;
        uint8_t s;
        while (afsk_mod_next(&mod, &s))
            buf[n++] = (s - 128) / 127.0f;
    }
    for (int i = 0; i < TX_FS / 4; i++)
        buf[n++] = 0;
    *out_len = n;
    return buf;
}

static int run(const scenario_t *sc, const float *tx, int tx_len, const char *trace_path)
{
    static afsk_demod_t d;
    afsk_demod_trace_t tr;
    FILE *tf = NULL;

    afsk_demod_init(&d, on_frame, NULL);
    if (trace_path)
    {
        tf = fopen(trace_path, "w");
        fprintf(tf, "n,in,bpf,mark,space,demod,pll,sampled\n");
        d.trace = &tr;
    }
    decoded_count = 0;
    memset(decoded_match, 0, sizeof(decoded_match));

    /* Twist: one-pole highpass/lowpass blend applied at the TX rate. */
    double g = pow(10.0, sc->twist_db / 20.0);
    double lp = 0.0, a = 1.0 - exp(-2.0 * 3.14159265358979 * 1700.0 / TX_FS);

    float *shaped = malloc(tx_len * sizeof(float));
    for (int i = 0; i < tx_len; i++)
    {
        lp += a * (tx[i] - lp);
        double hp = tx[i] - lp;
        /* g > 1 boosts highs (space) relative to lows (mark) by roughly g at the tone pair */
        shaped[i] = (float)((lp + g * hp) / (g > 1.0 ? g : 1.0));
    }

    double ratio = (double)TX_FS / (AFSK_DEMOD_FS_IN * (1.0 + sc->adc_ppm * 1e-6));
    int n_out = (int)((tx_len - 2) / ratio);
    int trace_n = 0;
    long sig_n = 0, sig_dcd = 0, idle_n = 0, idle_dcd = 0;
    int last_active = -1000000, burst_start = 0;
    for (int k = 0; k < n_out; k++)
    {
        double t = k * ratio;
        int i = (int)t;
        double f = t - i;
        double v = shaped[i] * (1.0 - f) + shaped[i + 1] * f;
        double adc = 2048 + sc->adc_offset + v * sc->amplitude + gauss() * sc->noise_rms;
        int code = (int)lround(adc);
        if (code < 0)
            code = 0;
        if (code > 4095)
            code = 4095;

        afsk_demod_process(&d, (code - 2048) / 2048.0f);

        /* carrier detect statistics: skip 30 ms after a burst starts and 50 ms after it ends */
        if (tx[i] != 0.0f)
        {
            if (k - last_active > TX_FS / 100)
                burst_start = k;
            last_active = k;
        }
        int in_burst = k - last_active < AFSK_DEMOD_FS_IN / 200;
        if (in_burst && k - burst_start > AFSK_DEMOD_FS_IN * 3 / 100)
        {
            sig_n++;
            sig_dcd += d.dcd;
        }
        else if (!in_burst && k - last_active > AFSK_DEMOD_FS_IN / 20)
        {
            idle_n++;
            idle_dcd += d.dcd;
        }

        if (wav_out)
        {
            int16_t s = (int16_t)((code - 2048) * 16); /* same scaling as the ESP32 48k tap */
            fwrite(&s, 2, 1, wav_out);
        }

        if (tf && (k % AFSK_DEMOD_DECIM) == 0 && trace_n < AFSK_DEMOD_FS * 2)
            fprintf(tf, "%d,%g,%g,%g,%g,%g,%ld,%d\n", trace_n++, tr.in, tr.bpf, tr.mark, tr.space,
                    tr.demod, (long)tr.pll, tr.sampled);
    }
    free(shaped);
    if (tf)
        fclose(tf);

    dcd_on_signal = sig_n ? 100.0 * sig_dcd / sig_n : 0;
    dcd_on_idle = idle_n ? 100.0 * idle_dcd / idle_n : 0;

    int ok = 0;
    for (int i = 0; i < NMSG; i++)
        ok += decoded_match[i] > 0;
    return ok;
}

/* Several frames in one transmission, one flag apart (what the ESP32 does with a queue of frames). */
static int test_burst(void)
{
    static uint8_t bits[4096];
    static afsk_mod_t mod;
    static afsk_demod_t d;
    afsk_encoder_t e;

    afsk_enc_init(&e, bits, sizeof(bits));
    afsk_enc_flags(&e, 30);
    for (int m = 0; m < NMSG; m++)
    {
        uint8_t frame[AX25_MAX_FRAME];
        int flen = ax25_from_tnc2(messages[m], frame, sizeof(frame));
        if (m)
            afsk_enc_flags(&e, 1);
        if (afsk_enc_frame(&e, frame, flen) < 0)
            return -1;
    }
    afsk_enc_flags(&e, 3);

    afsk_mod_init(&mod, TX_FS, 100);
    afsk_mod_load(&mod, bits, e.nbits);
    afsk_demod_init(&d, on_frame, NULL);
    decoded_count = 0;
    memset(decoded_match, 0, sizeof(decoded_match));

    /* 40 kHz -> 48 kHz by linear interpolation, plus a little silence on both sides */
    float prev = 0.0f, cur = 0.0f;
    double t = 0.0;
    uint8_t s;
    int more = 1;
    for (int k = 0; k < AFSK_DEMOD_FS_IN / 10; k++)
        afsk_demod_process(&d, 0.0f);
    while (more)
    {
        while (t >= 1.0)
        {
            prev = cur;
            more = afsk_mod_next(&mod, &s);
            cur = more ? (s - 128) / 127.0f * 0.4f : 0.0f;
            t -= 1.0;
        }
        afsk_demod_process(&d, prev + (cur - prev) * (float)t);
        t += (double)TX_FS / AFSK_DEMOD_FS_IN;
    }
    for (int k = 0; k < AFSK_DEMOD_FS_IN / 10; k++)
        afsk_demod_process(&d, 0.0f);

    int ok = 0;
    for (int i = 0; i < NMSG; i++)
        ok += decoded_match[i] > 0;
    return ok;
}

int main(int argc, char **argv)
{
    int trace = 0;
    const char *wav_in = NULL, *wav_make = NULL, *trace_path = NULL;
    for (int i = 1; i < argc; i++)
    {
        if (!strcmp(argv[i], "--trace"))
        {
            trace = 1;
            if (i + 1 < argc && argv[i + 1][0] != '-')
                trace_path = argv[++i];
        }
        else if (!strcmp(argv[i], "--wav") && i + 1 < argc)
            wav_in = argv[++i];
        else if (!strcmp(argv[i], "--make-wav") && i + 1 < argc)
            wav_make = argv[++i];
        else
        {
            printf("usage: %s [--trace [file.csv]] [--wav capture.wav] [--make-wav out.wav]\n", argv[0]);
            return 2;
        }
    }
    if (wav_in)
        return replay_wav(wav_in, trace ? (trace_path ? trace_path : "afsk_demod_trace.csv") : NULL);

    int tx_len;
    float *tx = build_tx(&tx_len);
    srand(1234);

    /* sanity: TNC2 round trip */
    for (int i = 0; i < NMSG; i++)
    {
        uint8_t f[AX25_MAX_FRAME];
        char back[600];
        int n = ax25_from_tnc2(messages[i], f, sizeof(f));
        ax25_to_tnc2(f, n - 2, back, sizeof(back));
        if (strcmp(back, messages[i]) != 0)
        {
            printf("TNC2 round trip mismatch:\n  %s\n  %s\n", messages[i], back);
            return 2;
        }
    }

    const struct
    {
        const char *name;
        scenario_t sc;
        int must_pass;
    } tests[] = {
        {"clean, full scale", {0, 1500, 0, 0, 0}, 1},
        {"clean, small (+-100 counts)", {0, 100, 1, 0, 0}, 1},
        {"DC offset +600 counts", {0, 800, 2, 0, 600}, 1},
        {"ADC clock +1000 ppm", {1000, 800, 2, 0, 0}, 1},
        {"ADC clock -1000 ppm", {-1000, 800, 2, 0, 0}, 1},
        {"twist -6 dB (de-emphasis)", {0, 800, 2, -6, 0}, 1},
        {"twist +6 dB (pre-emphasis)", {0, 800, 2, 6, 0}, 1},
        {"noise, SNR ~20 dB", {300, 800, 57, 0, 0}, 1},
        {"noise, SNR ~12 dB", {300, 800, 140, 0, 0}, 1},
        {"noise, SNR ~8 dB", {300, 800, 225, 0, 0}, 0},
        {"noise, SNR ~6 dB", {300, 800, 283, 0, 0}, 0},
        {"noise, SNR ~3 dB", {300, 800, 400, 0, 0}, 0},
        {"noise, SNR ~0 dB", {300, 800, 566, 0, 0}, 0},
        {"twist -10 dB + SNR ~12 dB", {0, 800, 140, -10, 0}, 0},
        {"twist +10 dB + SNR ~12 dB", {0, 800, 140, 10, 0}, 0},
        {"ADC clock +5000 ppm", {5000, 800, 2, 0, 0}, 0},
    };

    if (wav_make)
    {
        wav_out = fopen(wav_make, "wb");
        if (!wav_out)
            return 2;
        wav_write_header(wav_out, AFSK_DEMOD_FS_IN, 0);
        int ok = run(&tests[8].sc, tx, tx_len, NULL); /* "noise, SNR ~12 dB" */
        long bytes = ftell(wav_out) - 44;
        wav_write_header(wav_out, AFSK_DEMOD_FS_IN, (uint32_t)bytes);
        fclose(wav_out);
        printf("wrote %s (%s, %.1f s), %d/%d frames decoded while writing\n", wav_make, tests[8].name,
               bytes / 2.0 / AFSK_DEMOD_FS_IN, ok, NMSG);
        return 0;
    }

    int failed = 0;
    int burst = test_burst();
    printf("%-30s %d/%d frames%s\n", "burst, 1 flag between frames", burst, NMSG, burst == NMSG ? "" : "  <-- FAIL");
    if (burst != NMSG)
        failed++;

    for (size_t t = 0; t < sizeof(tests) / sizeof(tests[0]); t++)
    {
        int ok = run(&tests[t].sc, tx, tx_len, (trace && t == 0) ? "afsk_demod_trace.csv" : NULL);
        int pass = ok == NMSG;
        printf("%-30s %d/%d frames   DCD on: signal %5.1f%%  idle %5.1f%%%s\n", tests[t].name, ok, NMSG,
               dcd_on_signal, dcd_on_idle,
               pass ? "" : (tests[t].must_pass ? "  <-- FAIL" : "  (informational)"));
        if (!pass && tests[t].must_pass)
            failed++;
    }
    free(tx);
    printf(failed ? "FAILED\n" : "PASS\n");
    return failed ? 1 : 0;
}
