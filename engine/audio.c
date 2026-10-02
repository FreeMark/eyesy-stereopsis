/* audio.c - stereopsis's audio analysis. It runs in the engine's audio process (sound.py) on every block the
 * sound card delivers (32 kHz, interleaved S16). The spectrum is scaled by the same gain the stock scope data
 * gets (Shift + knob 1), without the clipping that gain causes in the scope data.
 *
 * Every HOP samples (512 = 16 ms at 32 kHz) it analyses the latest N (2048 = 64 ms) the way a browser's
 * AnalyserNode does (Blackman window, |X[k]| / N, smoothing over time 0.6, dB mapped from -100..-30 to 0..1),
 * so visuals written against Web Audio (free.vet's visualizers) see the same numbers. From that spectrum:
 *   - free.vet's band envelopes: level, bass (20-200 Hz), mid (200-2000), treble (2000-12000), 0..1, fast
 *     to rise and slower to fall
 *   - kicks (kick_step): the 150 Hz low-passed energy of the last 16 ms well above its running average, looked at
 *     every 4 ms, 0.12 s refractory; a beat counter and a beat envelope (1 on a kick, decaying over ~0.3 s)
 *   - 32 log-spaced display bands (50 Hz - 16 kHz)
 *   - a tempo estimate: autocorrelation of the spectral-flux onset curve over the last 8 s, once a second,
 *     reported only while that window also holds 4+ kicks and the estimates keep agreeing (tempo_vote)
 * Nothing here allocates after au_init, and every entry point checks its arguments: a bad call returns a
 * negative code, never crashes the audio process.
 */
#include <errno.h>
#include <math.h>
#include <stdint.h>
#include <string.h>

#define AU_VERSION 2
#define N 2048                    /* analysis window */
#define HALF (N / 2)              /* spectrum bins (bin 0 = DC) */
#define HOP 512                   /* samples between analyses */
#define KSUB 128                  /* the kick detector looks every KSUB samples (4 ms) ... */
#define KSTEPS (HOP / KSUB)       /* ... at the last KSTEPS of them (16 ms) */
#define NBANDS 32                 /* display bands */
#define OH 512                    /* onset history for the tempo (8.2 s at 62.5 analyses a second) */
#define NVALS 16                  /* the values vector, see au_results */

static struct {
    int ready;
    float rate, bin_hz, dt;
    float ring[N];                /* the latest N mono samples, circular */
    int wpos, since_hop;
    float win[N], cosw[HALF], sinw[HALF];
    uint16_t rev[N];
    float re[N], im[N];
    float smooth[HALF];           /* smoothed linear magnitudes (Web Audio's X^) */
    float fft01[HALF], prev01[HALF];
    int blo[NBANDS], bhi[NBANDS];
    float bands[NBANDS];
    float level, bass, mid, treble, beat_env, bass_avg, cooldown, rms;
    uint32_t seq, beats;
    float onset[OH];
    int opos, ofill, since_tempo;
    float bpm, bpm_conf, bpm_pending, silent_s, onset_sd;
    int pending_hits, support, armed;
    uint32_t kick_seq[8];         /* analysis number of the last 8 kicks */
    float kick_avg;               /* the kick detector: low-passed energy per 16 ms, its running average */
    float lp_b0, lp_b1, lp_b2, lp_a1, lp_a2, lp_x1, lp_x2, lp_y1, lp_y2;   /* 150 Hz biquad low-pass */
    double lp_e;                  /* its energy since the last look */
    double ksub[KSTEPS];          /* the energy of the last KSTEPS looks */
    int kpos, ksince, below;     /* below: looks in a row with the energy back down (re-arm) */
    float gain;                   /* the latest gain (the ring holds raw samples) */
} A;

int au_version(void) {
    return AU_VERSION;
}

/* rate = the sound card's sample rate (32000 on the EYESY). 0 or -EINVAL. */
int au_init(int rate) {
    if (rate < 8000 || rate > 192000)
        return -EINVAL;
    memset(&A, 0, sizeof A);
    A.rate = (float)rate;
    A.bin_hz = (float)rate / N;
    A.dt = (float)HOP / (float)rate;
    const double tau = 6.283185307179586;
    for (int n = 0; n < N; n++)                      /* Blackman, alpha 0.16, as Web Audio's analyser */
        A.win[n] = (float)(0.42 - 0.5 * cos(tau * n / N) + 0.08 * cos(2.0 * tau * n / N));
    for (int k = 0; k < HALF; k++) {
        A.cosw[k] = (float)cos(tau * k / N);
        A.sinw[k] = (float)-sin(tau * k / N);
    }
    int bits = 0;
    while ((1 << bits) < N)
        bits++;
    for (int i = 0; i < N; i++) {
        int r = 0;
        for (int b = 0; b < bits; b++)
            r |= ((i >> b) & 1) << (bits - 1 - b);
        A.rev[i] = (uint16_t)r;
    }
    double f0 = 50.0, f1 = 16000.0;                  /* display bands, log-spaced */
    for (int b = 0; b < NBANDS; b++) {
        double lo = f0 * pow(f1 / f0, (double)b / NBANDS), hi = f0 * pow(f1 / f0, (double)(b + 1) / NBANDS);
        int klo = (int)floor(lo / A.bin_hz + 0.5), khi = (int)floor(hi / A.bin_hz + 0.5);
        if (klo < 1)
            klo = 1;
        if (khi > HALF)
            khi = HALF;
        if (khi <= klo)
            khi = klo + 1 <= HALF ? klo + 1 : HALF;
        if (klo >= HALF)
            klo = HALF - 1;
        A.blo[b] = klo;
        A.bhi[b] = khi;
    }
    {                                                /* RBJ cookbook low-pass, 150 Hz, Q 0.707 */
        double w0 = 2.0 * 3.141592653589793 * 150.0 / rate, al = sin(w0) / (2.0 * 0.7071), c = cos(w0);
        double a0 = 1.0 + al;
        A.lp_b0 = (float)((1.0 - c) / 2.0 / a0);
        A.lp_b1 = (float)((1.0 - c) / a0);
        A.lp_b2 = A.lp_b0;
        A.lp_a1 = (float)(-2.0 * c / a0);
        A.lp_a2 = (float)((1.0 - al) / a0);
    }
    A.armed = 1;
    A.ready = 1;
    return 0;
}

static void fft(void) {
    for (int i = 0; i < N; i++) {
        int j = A.rev[i];
        if (j > i) {
            float t = A.re[i];
            A.re[i] = A.re[j];
            A.re[j] = t;
            t = A.im[i];
            A.im[i] = A.im[j];
            A.im[j] = t;
        }
    }
    for (int len = 2; len <= N; len <<= 1) {
        int half = len >> 1, step = N / len;
        for (int s = 0; s < N; s += len)
            for (int k = 0; k < half; k++) {
                float wr = A.cosw[k * step], wi = A.sinw[k * step];
                int a = s + k, b = a + half;
                float xr = A.re[b] * wr - A.im[b] * wi, xi = A.re[b] * wi + A.im[b] * wr;
                A.re[b] = A.re[a] - xr;
                A.im[b] = A.im[a] - xi;
                A.re[a] += xr;
                A.im[a] += xi;
            }
    }
}

static float band_mean(float lo_hz, float hi_hz) {
    int lo = (int)floorf(lo_hz / A.bin_hz), hi = (int)ceilf(hi_hz / A.bin_hz);   /* as free.vet's bandEnergy */
    if (lo < 1)
        lo = 1;
    if (hi > HALF)
        hi = HALF;
    if (hi <= lo)
        return 0.0f;
    float s = 0.0f;
    for (int k = lo; k < hi; k++)
        s += A.fft01[k];
    return s / (float)(hi - lo);
}

static float follow(float prev, float tgt, float dt) {    /* fast to rise, slower to fall */
    float k = tgt > prev ? dt * 20.0f : dt * 6.0f;
    if (k > 1.0f)
        k = 1.0f;
    return prev + (tgt - prev) * k;
}

/* one estimate a second (0 = none). The reported tempo lives on support: an estimate that agrees with it
 * (within 4 %) adds one (up to 4), any other takes one away, and at 0 the tempo is dropped. A new tempo is
 * taken after two agreeing estimates in a row. Measured on the EYESY's own input noise (-50 dBFS): its
 * autocorrelation peaks wander, and without this a tempo, once set, never went away. */
static void tempo_vote(float bpm) {
    if (A.bpm > 0.0f && bpm > 0.0f && fabsf(bpm - A.bpm) < A.bpm * 0.04f) {
        A.bpm += (bpm - A.bpm) * 0.3f;
        if (A.support < 4)
            A.support++;
        A.pending_hits = 0;
        return;
    }
    if (A.support > 0 && --A.support == 0)
        A.bpm = 0.0f;
    if (bpm <= 0.0f) {
        A.pending_hits = 0;
        return;
    }
    if (A.pending_hits > 0 && fabsf(bpm - A.bpm_pending) < A.bpm_pending * 0.04f) {
        if (A.bpm <= 0.0f) {                          /* seen twice in a row, and no tempo holds */
            A.bpm = 0.5f * (bpm + A.bpm_pending);
            A.support = 2;
            A.pending_hits = 0;
            return;
        }
    }
    A.bpm_pending = bpm;
    A.pending_hits = 1;
}

static void tempo(void) {
    int n = A.ofill < OH ? A.ofill : OH;
    if (n < OH / 2)                                   /* ~4 s of history before the first estimate */
        return;
    static float x[OH];
    float mean = 0.0f;
    for (int i = 0; i < n; i++) {
        x[i] = A.onset[(A.opos - n + i + OH) % OH];
        mean += x[i];
    }
    mean /= (float)n;
    float e0 = 0.0f;
    for (int i = 0; i < n; i++) {
        x[i] -= mean;
        e0 += x[i] * x[i];
    }
    A.onset_sd = sqrtf(e0 / (float)n);
    /* the autocorrelation is normalised, so a steady tone's faint ripple (its period beating against the hop)
     * reads as a confident tempo. Measured: steady tones 0.00002, white noise 0.0006, kick drums 0.0012
     * (at -30 dB) to 0.007: below this floor there is no rhythm to read */
    if (A.onset_sd < 2e-4f) {
        A.bpm = A.bpm_conf = 0.0f;
        A.pending_hits = A.support = 0;
        return;
    }
    /* and it needs kicks: at least 4 in the same window. The EYESY's own input noise has enough periodic
     * structure to pass everything above now and then (a brief "137 BPM" with nothing playing); it
     * produced one kick in 30 s. So: a tempo for music with a beat, which is what beat-synced visuals use */
    int kicks = 0;
    for (int i = 0; i < 8; i++)
        if (A.kick_seq[i] && A.seq + 1 - A.kick_seq[i] < (uint32_t)n)
            kicks++;
    if (kicks < 4) {
        A.bpm_conf = 0.0f;
        tempo_vote(0.0f);
        return;
    }
    float fps = 1.0f / A.dt;
    int lmin = (int)floorf(fps * 60.0f / 200.0f), lmax = (int)ceilf(fps * 60.0f / 60.0f);   /* 200..60 BPM */
    if (lmax >= n - 1)
        lmax = n - 2;
    if (e0 <= 1e-9f || lmin < 2 || lmax <= lmin + 1) {
        A.bpm_conf = 0.0f;
        return;
    }
    static float ac[OH];
    int best = -1;
    float best_w = 0.0f;
    for (int L = lmin - 1; L <= lmax + 1; L++) {
        float s = 0.0f;
        for (int i = L; i < n; i++)
            s += x[i] * x[i - L];
        ac[L] = s / e0;
    }
    for (int L = lmin; L <= lmax; L++) {
        float bpm = 60.0f * fps / (float)L;
        float o = log2f(bpm / 120.0f);                /* prefer tempos near 120 (octave errors) */
        float w = ac[L] * expf(-0.5f * o * o);
        if (ac[L] > ac[L - 1] && ac[L] >= ac[L + 1] && w > best_w) {
            best_w = w;
            best = L;
        }
    }
    if (best < 0 || ac[best] < 0.2f) {                /* no estimate this second */
        A.bpm_conf = best < 0 ? 0.0f : ac[best];
        tempo_vote(0.0f);
        return;
    }
    float y0 = ac[best - 1], y1 = ac[best], y2 = ac[best + 1], d = y0 - 2.0f * y1 + y2;
    float off = d != 0.0f ? 0.5f * (y0 - y2) / d : 0.0f;   /* parabolic peak */
    if (off > 0.5f)
        off = 0.5f;
    if (off < -0.5f)
        off = -0.5f;
    A.bpm_conf = y1;
    tempo_vote(60.0f * fps / ((float)best + off));
}

static void analyse(void) {
    double e = 0.0;
    for (int n = 0; n < N; n++) {
        float s = A.ring[(A.wpos + n) % N];           /* oldest first, raw (before the gain) */
        e += (double)s * s;
        A.re[n] = s * A.win[n];
        A.im[n] = 0.0f;
    }
    A.rms = fminf((float)sqrt(e / N) * A.gain, 1.0f);
    fft();
    /* the gain scales the spectrum (the FFT is linear), so the visuals follow the knob as the scopes do -
     * but without the clipping the scope data gets: a loud kick on a loud bass line clipped flat is still a
     * kick here */
    const float inv_n = A.gain / N, tau = 0.6f, lo_db = -100.0f, hi_db = -30.0f;
    for (int k = 0; k < HALF; k++) {
        float mag = sqrtf(A.re[k] * A.re[k] + A.im[k] * A.im[k]) * inv_n;
        A.smooth[k] = tau * A.smooth[k] + (1.0f - tau) * mag;
        float db = A.smooth[k] > 0.0f ? 20.0f * log10f(A.smooth[k]) : -1000.0f;
        float v = (db - lo_db) / (hi_db - lo_db);
        A.fft01[k] = v < 0.0f ? 0.0f : v > 1.0f ? 1.0f : v;
    }
    float flux = 0.0f;
    for (int k = 1; k < HALF; k++) {
        float d = A.fft01[k] - A.prev01[k];
        if (d > 0.0f)
            flux += d;
        A.prev01[k] = A.fft01[k];
    }
    A.onset[A.opos] = flux / (float)(HALF - 1);
    A.opos = (A.opos + 1) % OH;
    if (A.ofill < OH)
        A.ofill++;
    for (int b = 0; b < NBANDS; b++) {
        float s = 0.0f;
        for (int k = A.blo[b]; k < A.bhi[b]; k++)
            s += A.fft01[k];
        A.bands[b] = s / (float)(A.bhi[b] - A.blo[b]);
    }
    float dt = A.dt;
    float ib = band_mean(20, 200), im = band_mean(200, 2000), it = band_mean(2000, 12000), il = band_mean(20, 12000);
    A.bass = follow(A.bass, ib, dt);
    A.mid = follow(A.mid, im, dt);
    A.treble = follow(A.treble, it, dt);
    A.level = follow(A.level, il, dt);
    float ka = dt * 1.5f;
    A.bass_avg += (ib - A.bass_avg) * (ka > 1.0f ? 1.0f : ka);
    A.silent_s = A.rms < 1e-3f ? A.silent_s + dt : 0.0f;     /* quieter than -60 dBFS */
    if (A.silent_s > 4.0f) {                                   /* the music stopped: forget its tempo */
        A.bpm = A.bpm_conf = 0.0f;
        A.pending_hits = A.support = 0;
    } else if (++A.since_tempo * dt >= 1.0f) {
        A.since_tempo = 0;
        tempo();
    }
    A.seq++;
}

/* kicks. free.vet's detector (the dB-mapped bass 1.4x above its running average) goes blind when the music is loud:
 * every bass bin sits at the -30 dB ceiling (measured on Free's unit, music at -5..-12 dBFS after the gain: one beat
 * in 30 kicks), and the 64 ms window smears a kick's attack over four hops. So, in the time domain: the raw signal
 * (before the gain, so never clipped) through a 150 Hz low-pass, its energy over the last 16 ms 2x above its running
 * average (1.5/s), over a floor far above input noise (Free's unit, nothing playing: ~6e-9; quiet drums at -30 dB:
 * ~4e-4). A kick on a loud bass line is still a jump of several times. Plus a re-arm: after a kick the energy must
 * fall back near its average first (for a whole 16 ms), or the kick's tail fires again once the 0.12 s refractory
 * is over.
 * v2: it looks every 4 ms (KSUB) at the last 16 ms instead of once per 16 ms hop, so a kick is found as soon as its
 * energy is in (it used to wait for the end of the hop it fell in: 0-16 ms later), and sound.py publishes the beat
 * at once instead of with the next analysis */
static void kick_step(void) {
    A.ksub[A.kpos] = A.lp_e;
    A.kpos = (A.kpos + 1) % KSTEPS;
    A.lp_e = 0.0;
    double e = 0.0;
    for (int i = 0; i < KSTEPS; i++)
        e += A.ksub[i];
    float kp = (float)(e / HOP);
    float dk = (float)KSUB / A.rate, ka = dk * 1.5f;
    A.kick_avg += (kp - A.kick_avg) * (ka > 1.0f ? 1.0f : ka);
    A.cooldown -= dk;
    if (A.cooldown < 0.0f)
        A.cooldown = 0.0f;
    if (!A.armed) {                                  /* re-armed once the energy stayed back down for 16 ms */
        A.below = kp < A.kick_avg * 1.3f ? A.below + 1 : 0;
        if (A.below >= KSTEPS) {
            A.armed = 1;
            A.below = 0;
        }
    }
    if (A.armed && kp > A.kick_avg * 2.0f && kp > 1e-6f && A.cooldown <= 0.0f) {
        A.beat_env = 1.0f;
        A.cooldown = 0.12f;
        A.kick_seq[A.beats % 8] = A.seq + 1;          /* +1: 0 means "no kick" */
        A.beats++;
        A.armed = 0;
    } else {
        A.beat_env -= dk * 3.5f;
        if (A.beat_env < 0.0f)
            A.beat_env = 0.0f;
    }
}

/* frames of interleaved S16 (channels 1 or 2, mixed to mono), and the gain the scope data gets (the
 * spectrum is scaled by it, the waveform output also clipped by it). Returns how many analyses ran (0 or
 * more), or a negative code. */
int au_push(const int16_t *pcm, int frames, int channels, float gain) {
    if (!A.ready)
        return -ENODEV;
    if (!pcm || frames < 0 || frames > 65536 || channels < 1 || channels > 2 || !(gain >= 0.0f && gain <= 1e4f))
        return -EINVAL;
    A.gain = gain;
    const float scale = 1.0f / 32768.0f;
    int ran = 0;
    for (int f = 0; f < frames; f++) {
        float s = channels == 2 ? 0.5f * ((float)pcm[2 * f] + (float)pcm[2 * f + 1]) : (float)pcm[f];
        s *= scale;
        float y = A.lp_b0 * s + A.lp_b1 * A.lp_x1 + A.lp_b2 * A.lp_x2 - A.lp_a1 * A.lp_y1 - A.lp_a2 * A.lp_y2;
        A.lp_x2 = A.lp_x1;
        A.lp_x1 = s;
        A.lp_y2 = A.lp_y1;
        A.lp_y1 = y;
        A.lp_e += (double)y * y;
        if (++A.ksince >= KSUB) {
            A.ksince = 0;
            kick_step();
        }
        A.ring[A.wpos] = s;
        A.wpos = (A.wpos + 1) % N;
        if (++A.since_hop >= HOP) {
            A.since_hop = 0;
            analyse();
            ran++;
        }
    }
    return ran;
}

/* The results, copied out (sound.py copies them into shared memory under the engine's audio lock):
 *   vals[NVALS]: 0 seq (mod 1e6), 1 level, 2 bass, 3 mid, 4 treble, 5 beat envelope, 6 beat count (mod 1e6),
 *     7 bpm (0 = no estimate yet), 8 tempo confidence 0..1, 9 rms of the window (0..1), 10 bin width Hz,
 *     11 analyses per second, 12 onset activity (spread of the onset curve the tempo is read from),
 *     13..15 reserved (0)
 *   bands[nbands <= 32]: the display bands, 0..1
 *   fft[nfft <= 1024]: the 0..1 spectrum, bin k = k * bin width
 *   wave[nwave <= 2048]: the newest nwave mono samples, oldest first, -1..1
 * Any pointer may be NULL (skipped). Returns 0, or a negative code (nothing written). */
int au_results(float *vals, int nvals, float *bands, int nbands, float *fft01, int nfft, float *wave, int nwave) {
    if (!A.ready)
        return -ENODEV;
    if (nvals < 0 || nvals > NVALS || nbands < 0 || nbands > NBANDS || nfft < 0 || nfft > HALF || nwave < 0 || nwave > N)
        return -EINVAL;
    if (vals) {
        float v[NVALS] = { (float)(A.seq % 1000000u), A.level, A.bass, A.mid, A.treble, A.beat_env,
                           (float)(A.beats % 1000000u), A.bpm, A.bpm_conf, A.rms, A.bin_hz, 1.0f / A.dt,
                           A.onset_sd, 0, 0, 0 };
        memcpy(vals, v, sizeof(float) * (size_t)nvals);
    }
    if (bands)
        memcpy(bands, A.bands, sizeof(float) * (size_t)nbands);
    if (fft01)
        memcpy(fft01, A.fft01, sizeof(float) * (size_t)nfft);
    if (wave)                                          /* as the scope data: the gain, clipped to full scale */
        for (int i = 0; i < nwave; i++)
            wave[i] = fminf(fmaxf(A.ring[(A.wpos - nwave + i + N) % N] * A.gain, -1.0f), 1.0f);
    return 0;
}
