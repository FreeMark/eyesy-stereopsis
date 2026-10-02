/* phase.c - the kernel of the EYESY mode "S - Phase Space": the live waveform as a curve in three dimensions, an
 * oscilloscope's phosphor trace in the Swarm Visualizer's light (glow.h).
 *
 * Each point of the curve is the signal now, a moment ago and two moments ago: (x(t), x(t - tau), x(t - 2 tau)) - a
 * delay embedding, the phase space a dynamical system is drawn in. tau follows the music's pitch (a quarter of the
 * period the waveform's autocorrelation finds): a pure tone traces a circle, an octave a figure-eight, a chord a
 * knot, noise a glowing cloud, a kick a burst from the middle. Knob 1 shortens or stretches tau around that.
 * The beam is drawn as an electron beam lights a screen: brightest where it moves slowest, the newest part hottest,
 * phosphor persistence on knob 3 (the trails). The figure is auto-scaled to the loudness, turns on knob 2 (centre
 * still), swells on kicks; Trigger strobes it. A faint box of axes gives it depth. */
#include "glow.h"

#define PS_VERSION 1
#define MAXS 4096                  /* the waveform history kept, samples */
#define MAXSEG 700                 /* curve segments drawn a frame */

enum {
    P_DT, P_K1, P_K2, P_K3, P_K4, P_K5,
    P_FG_R, P_FG_G, P_FG_B, P_BG_R, P_BG_G, P_BG_B,
    P_LEVEL, P_BASS, P_MID, P_TREBLE, P_BEAT, P_KICK, P_TRIG, P_PREFILLED,
    P_WAVE_HZ,
    P_COUNT
};

static struct {
    int ready;
    float tau;                     /* the delay, seconds (smoothed) */
    float scale, rms;              /* auto gain */
    float t, yaw, pitch, kick_glow, strobe, spin_x;
} Q;

static float g_x[MAXS];            /* this frame's samples, oldest first */

int fx_version(void) { return PS_VERSION; }
int fx_param_count(void) { return P_COUNT; }
int fx_ready(void) { return Q.ready; }
int fx_threads(int n) { return gl_set_threads(n); }
void fx_trails_reset(void) { gl_trails_reset(); }
float fx_bench(int which, int n, float r) { return gl_bench(which, n, r); }

int fx_init(int n, int seed) {
    (void)n;
    (void)seed;
    memset(&Q, 0, sizeof Q);
    Q.tau = 1.0f / (4.0f * 110.0f);
    Q.scale = 1.0f;
    Q.rms = 0.1f;
    Q.yaw = 0.5f;
    Q.pitch = 0.35f;
    Q.ready = 1;
    return 0;
}

/* the fundamental's period, seconds, from the autocorrelation of the waveform decimated to ~8 kHz (0 = none clear) */
static float period_of(const float *x, int n, float hz) {
    int dec = hz > 16000.0f ? (int)(hz / 8000.0f) : 1;
    float d[1024];
    int m = 0;
    for (int i = 0; i + dec <= n && m < 1024; i += dec) {
        float s = 0.0f;
        for (int k = 0; k < dec; k++) s += x[i + k];
        d[m++] = s / (float)dec;
    }
    float dhz = hz / (float)dec;
    int lo = (int)(dhz / 1000.0f), hi = (int)(dhz / 55.0f);   /* 55 Hz .. 1 kHz */
    lo = lo < 2 ? 2 : lo;
    hi = hi > m / 2 ? m / 2 : hi;
    if (hi <= lo + 2) return 0.0f;
    float e0 = 0.0f;
    for (int i = 0; i < m; i++) e0 += d[i] * d[i];
    if (e0 < 1e-5f) return 0.0f;
    /* the first lag whose normalised autocorrelation peaks above 0.5 (the fundamental, not a multiple) */
    float prev = 1.0f, best = 0.0f;
    int best_lag = 0, climbing = 0;
    for (int lag = lo; lag <= hi; lag++) {
        float s = 0.0f;
        for (int i = 0; i + lag < m; i++) s += d[i] * d[i + lag];
        float r = s / e0 * (float)m / (float)(m - lag);
        if (r > prev) climbing = 1;
        else if (climbing && prev > 0.5f) { best_lag = lag - 1; best = prev; break; }
        prev = r;
    }
    (void)best;
    return best_lag > 0 ? (float)best_lag / dhz : 0.0f;
}

static GCam g_cam;

int fx_frame(uint8_t *dst, int dW, int dH, int dpitch, int down, int sh_r, int sh_g, int sh_b, float *acc, int W, int H,
             float *bloom, int BW, int BH, const float *fft, int nfft, float fft_hz, const float *wave, int nwave,
             const float *bands, int nbands, const float *P) {
    (void)fft_hz;
    (void)bands;
    if (!Q.ready) return -9;
    if (!P) return -1;
    if (nfft < 0 || nfft > 65536 || nwave < 0 || nwave > 65536 || nbands < 0 || nbands > 256 || (nfft > 0 && !fft) ||
        (nwave > 0 && !wave) || (nbands > 0 && !bands))
        return -3;
    const float k3 = gl_c01(P[P_K3]);
    int rc = gl_begin(dst, dW, dH, dpitch, down, sh_r, sh_g, sh_b, acc, W, H, bloom, BW, BH, k3 > 0.0f);
    if (rc) return rc;
    const float dt = gl_clampf(P[P_DT], 0.0f, 0.1f);
    Q.t += dt;
    if (Q.t > 1.0e5f) Q.t = 0.0f;
    const float hz = gl_clampf(P[P_WAVE_HZ], 100.0f, 200000.0f);
    int n = nwave < MAXS ? nwave : MAXS;
    for (int i = 0; i < n; i++) g_x[i] = gl_clampf(wave[nwave - n + i], -1.0f, 1.0f);
    /* two gentle poles at ~900 Hz: hats and hiss would scribble over the shape the bass and mids draw */
    if (n > 1) {
        const float a1 = 1.0f - expf(-6.2831853f * 900.0f / hz);
        float s1 = g_x[0], s2 = g_x[0];
        for (int i = 0; i < n; i++) {
            s1 += (g_x[i] - s1) * a1;
            s2 += (s1 - s2) * a1;
            g_x[i] = s2;
        }
    }
    /* auto gain: the signal's RMS, quick up, slow down */
    float e = 0.0f;
    for (int i = 0; i < n; i++) e += g_x[i] * g_x[i];
    float rms = n ? sqrtf(e / (float)n) : 0.0f;
    Q.rms += (rms - Q.rms) * fminf(1.0f, dt * (rms > Q.rms ? 8.0f : 1.0f));
    float gain = 1.0f / fmaxf(Q.rms * 1.5f, 0.02f);
    Q.scale += (gain - Q.scale) * fminf(1.0f, dt * 3.0f);
    /* tau: a quarter of the fundamental's period (followed smoothly); knob 1 from a quarter of it to four times */
    float per = n > 64 ? period_of(g_x, n, hz) : 0.0f;
    if (per > 0.0f) Q.tau += (per * 0.25f - Q.tau) * fminf(1.0f, dt * 4.0f);
    float k1 = (gl_c01(P[P_K1]) - 0.5f) * 2.0f;
    k1 = fabsf(k1) < 0.04f ? 0.0f : copysignf((fabsf(k1) - 0.04f) / 0.96f, k1);
    float tau_s = Q.tau * powf(4.0f, k1);
    int tau = (int)(tau_s * hz + 0.5f);
    tau = tau < 1 ? 1 : tau;
    if (2 * tau > n - 8) tau = (n - 8) / 2;
    if (tau < 1) tau = 1;                                        /* (a waveform under 10 samples: no curve below) */
    /* kicks swell the figure, Trigger strobes it */
    if (P[P_KICK] > 0.5f) Q.kick_glow = 1.0f;
    if (P[P_TRIG] > 0.5f) Q.strobe = 1.0f;
    Q.kick_glow = fmaxf(0.0f, Q.kick_glow - dt * 3.5f);
    Q.strobe = fmaxf(0.0f, Q.strobe - dt * 4.0f);
    /* the camera: knob 2 turns the figure (centre still); a slow tilt of its own */
    float k2 = (gl_c01(P[P_K2]) - 0.5f) * 2.0f;
    k2 = fabsf(k2) < 0.08f ? 0.0f : copysignf((fabsf(k2) - 0.08f) / 0.92f, k2);
    Q.yaw += k2 * 0.8f * dt;
    if (Q.yaw > 1.0e4f || Q.yaw < -1.0e4f) Q.yaw = 0.0f;
    Q.pitch = 0.35f + 0.2f * gl_sin(Q.t * 0.11f);
    gl_camera(&g_cam, Q.yaw, Q.pitch, 7.5f, 0.8f, 0.0f, 0.0f, 0.0f);

    float fg[3] = { gl_c01(P[P_FG_R]), gl_c01(P[P_FG_G]), gl_c01(P[P_FG_B]) };
    if (fg[0] + fg[1] + fg[2] < 0.15f) fg[0] = fg[1] = fg[2] = 0.7f;
    const float R = 2.0f;                                        /* the figure's half-size, world units */
    /* the box of axes */
    {
        float b = 0.05f + 0.05f * Q.kick_glow;
        float c[3] = { 0.5f * b + fg[0] * b, 0.55f * b + fg[1] * b, 0.6f * b + fg[2] * b };
        for (int a = 0; a < 3; a++) {
            float p0[3] = { 0, 0, 0 }, p1[3] = { 0, 0, 0 };
            p0[a] = -R * 1.1f;
            p1[a] = R * 1.1f;
            gl_line3(&g_cam, p0[0], p0[1], p0[2], p1[0], p1[1], p1[2], 0.012f, c, c);
        }
        for (int i = 0; i < 12; i++) {                           /* the cube's edges, fainter */
            int ax = i / 4, s1 = (i & 1) ? 1 : -1, s2 = (i & 2) ? 1 : -1;
            float p0[3], p1[3];
            p0[ax] = -R * 1.1f; p1[ax] = R * 1.1f;
            p0[(ax + 1) % 3] = p1[(ax + 1) % 3] = (float)s1 * R * 1.1f;
            p0[(ax + 2) % 3] = p1[(ax + 2) % 3] = (float)s2 * R * 1.1f;
            float cc[3] = { c[0] * 0.5f, c[1] * 0.5f, c[2] * 0.5f };
            gl_line3(&g_cam, p0[0], p0[1], p0[2], p1[0], p1[1], p1[2], 0.01f, cc, cc);
        }
    }
    /* the curve: the newest samples that have both delays behind them, thinned to <= MAXSEG segments */
    int first = 2 * tau, count = n - first;
    if (n >= 16 && count >= 2) {
        int step = (count + MAXSEG - 1) / MAXSEG;
        step = step < 1 ? 1 : step;
        const float k = Q.scale * R * (1.0f + 0.18f * Q.kick_glow);
        float px = 0, py = 0, pz = 0, sxp = 0, syp = 0;
        int have = 0;
        for (int i = first; i < n; i += step) {
            float x = gl_clampf(g_x[i] * k, -R * 1.6f, R * 1.6f);
            float y = gl_clampf(g_x[i - tau] * k, -R * 1.6f, R * 1.6f);
            float z = gl_clampf(g_x[i - 2 * tau] * k, -R * 1.6f, R * 1.6f);
            float sx, sy, d = gl_project(&g_cam, x, y, z, &sx, &sy);
            if (have && d > 0.2f) {
                /* a beam lights the screen by how long it stays: brightness ~ 1 / its speed, within limits; the
                 * newest part hotter */
                float len = sqrtf((sx - sxp) * (sx - sxp) + (sy - syp) * (sy - syp));
                float age = (float)(n - i) / (float)count;           /* 0 newest .. 1 oldest */
                float b = gl_clampf(4.0f / (len + 1.5f), 0.15f, 1.6f) * (1.3f - 0.8f * age);
                b *= 1.0f + 0.5f * Q.kick_glow + 2.0f * Q.strobe;
                float hot = (1.0f - age) * 0.35f + Q.strobe;
                float c[3] = { (fg[0] + hot) * b, (fg[1] + hot) * b, (fg[2] + hot) * b };
                gl_line3(&g_cam, px, py, pz, x, y, z, 0.03f, c, c);
            }
            px = x; py = y; pz = z;
            sxp = sx; syp = sy;
            have = d > 0.2f;
        }
    }
    /* knob 3: bloom up to the middle, a dead band, then phosphor persistence from 0.6 */
    const float bloom_amt = k3 < 0.5f ? k3 * 2.0f * 1.6f : 1.6f;
    const float decay = k3 >= 0.6f ? 0.45f + (k3 - 0.6f) * (0.45f / 0.4f) : 0.0f;
    const float bg[3] = { gl_c01(P[P_BG_R]), gl_c01(P[P_BG_G]), gl_c01(P[P_BG_B]) };
    gl_end(bloom_amt, decay, 1.0f, bg, P[P_PREFILLED] > 0.5f);
    return 0;
}

/* for tests and DEBUG: glow's stats, then tau (ms), the auto gain, the RMS */
void fx_stats(float *out, int n) {
    float v[11];
    gl_stats(v);
    v[8] = Q.tau * 1000.0f;
    v[9] = Q.scale;
    v[10] = Q.rms;
    for (int i = 0; i < n && i < 11; i++) out[i] = v[i];
}
