/* plasma.c - the kernel of the EYESY mode "11 - Plasma Globe": a plasma ball on the music, in the Swarm Visualizer's
 * light (glow.h).
 *
 * A glass sphere with an electrode in the middle; filaments of plasma run from it to the glass. Each filament belongs
 * to a band of the spectrum and burns with it; how many burn follows the loudness, and knob 1 (the voltage) raises
 * that and how wild they are. A filament is a curve from the electrode to the point where it meets the glass: it
 * bows sideways (a slow wander) and fizzes (a fine jitter that the treble raises), forks into a few short branches
 * near the glass, and lights a hot spot where it touches. The meeting points drift over the glass on their own, pulled
 * up a little (hot plasma rises). Kicks make every filament flare; Trigger is a hand on the glass - every filament
 * gathers to where it touches (up and to the right of the middle as you look), for as long as the Trigger is held
 * (P_HOLD), or a moment for a tap. The globe turns on knob 2 (centre still).
 * Version 2 (2026-09-30): the hand is the performer's alone (the mode passes the button / page / MIDI, not stock's
 * loudness trigger, which on a hot input kept the hand on the glass nonstop: every filament gathered at the point
 * facing you, the middle of the screen - "stuck in the center"); it touches off the axis, and lets go faster. */
#include "glow.h"

#define PG_VERSION 2
#define NFIL 16                    /* filaments (one per two bands) */
#define NSEG 22                    /* segments along a filament */

enum {
    P_DT, P_K1, P_K2, P_K3, P_K4, P_K5,
    P_FG_R, P_FG_G, P_FG_B, P_BG_R, P_BG_G, P_BG_B,
    P_LEVEL, P_BASS, P_MID, P_TREBLE, P_BEAT, P_KICK, P_TRIG, P_PREFILLED,
    P_WAVE_HZ,
    P_HOLD,                        /* v2: 1 while the Trigger button is held (the hand stays on the glass) */
    P_COUNT
};

typedef struct {
    float th, ph;                  /* where it meets the glass (spherical angles), drifting */
    float vth, vph;                /* its drift */
    float lvl;                     /* how brightly it burns (follows its band) */
    float on;                      /* 0..1: faded in / out as the loudness asks for more or fewer */
    uint32_t seed;
} Fil;

static struct {
    int ready;
    Fil f[NFIL];
    float t, yaw, kick_glow, touch, touch_th, touch_ph, fizz_t;
} G;

int fx_version(void) { return PG_VERSION; }
int fx_param_count(void) { return P_COUNT; }
int fx_ready(void) { return G.ready; }
int fx_threads(int n) { return gl_set_threads(n); }
void fx_trails_reset(void) { gl_trails_reset(); }
float fx_bench(int which, int n, float r) { return gl_bench(which, n, r); }

int fx_init(int n, int seed) {
    (void)n;
    memset(&G, 0, sizeof G);
    gl_rng_state = 0xB5297A4Du ^ ((uint32_t)seed * 2654435761u);
    if (!gl_rng_state) gl_rng_state = 1u;
    for (int i = 0; i < NFIL; i++) {
        Fil *f = &G.f[i];
        f->th = acosf(1.0f - 2.0f * gl_rnd());               /* uniform over the sphere */
        f->ph = gl_rnd() * 6.2831853f;
        f->vth = (gl_rnd() - 0.5f) * 0.5f;
        f->vph = (gl_rnd() - 0.5f) * 0.8f;
        f->seed = (uint32_t)(gl_rnd() * 4294967040.0f) | 1u;
    }
    G.ready = 1;
    return 0;
}

static GCam g_cam;

static void sph(float th, float ph, float r, float out[3]) {
    out[0] = r * sinf(th) * cosf(ph);
    out[1] = r * cosf(th);
    out[2] = r * sinf(th) * sinf(ph);
}

/* one filament (or a branch): from a to b, bowing by bow (world units) in the direction side, fizzing by fz, light
 * col (x brightness along it: brighter toward the glass); nseg segments */
static void strand(const float a[3], const float b[3], const float side[3], float bow, float fz, uint32_t seed,
                   const float col[3], float bright, float width, int nseg) {
    float d[3] = { b[0] - a[0], b[1] - a[1], b[2] - a[2] };
    float len = sqrtf(d[0] * d[0] + d[1] * d[1] + d[2] * d[2]) + 1e-6f;
    /* a second direction across (side x d): the fizz moves in both */
    float s2[3] = { side[1] * d[2] - side[2] * d[1], side[2] * d[0] - side[0] * d[2], side[0] * d[1] - side[1] * d[0] };
    float n2 = sqrtf(s2[0] * s2[0] + s2[1] * s2[1] + s2[2] * s2[2]) + 1e-6f;
    for (int k = 0; k < 3; k++) s2[k] /= n2;
    float prev[3] = { a[0], a[1], a[2] }, pc[3] = { 0, 0, 0 };
    for (int i = 1; i <= nseg; i++) {
        float s = (float)i / (float)nseg;
        float env = sinf(3.1415927f * s);                    /* 0 at both ends */
        float wob = bow * env * (0.8f + 0.2f * gl_sin(G.t * 1.7f + (float)(seed & 255)));
        /* the fizz: value noise along the strand, moving with time; finer toward the glass */
        float j1 = gl_noise1(s * 7.0f + G.fizz_t * 9.0f, seed) * fz * env;
        float j2 = gl_noise1(s * 7.0f + G.fizz_t * 9.0f + 31.0f, seed ^ 0x9E3779B9u) * fz * env;
        float p[3];
        for (int k = 0; k < 3; k++) p[k] = a[k] + d[k] * s + side[k] * (wob + j1) + s2[k] * j2;
        float b0 = bright * (0.45f + 0.75f * (s - 1.0f / nseg)), b1 = bright * (0.45f + 0.75f * s);
        float c0[3] = { col[0] * b0, col[1] * b0, col[2] * b0 }, c1[3] = { col[0] * b1, col[1] * b1, col[2] * b1 };
        if (i == 1) memcpy(pc, c0, sizeof pc);
        gl_line3(&g_cam, prev[0], prev[1], prev[2], p[0], p[1], p[2], width, i == 1 ? pc : c0, c1);
        memcpy(prev, p, sizeof prev);
    }
    (void)len;
}

int fx_frame(uint8_t *dst, int dW, int dH, int dpitch, int down, int sh_r, int sh_g, int sh_b, float *acc, int W, int H,
             float *bloom, int BW, int BH, const float *fft, int nfft, float fft_hz, const float *wave, int nwave,
             const float *bands, int nbands, const float *P) {
    (void)fft_hz;
    (void)wave;
    if (!G.ready) return -9;
    if (!P) return -1;
    if (nfft < 0 || nfft > 65536 || nwave < 0 || nwave > 65536 || nbands < 0 || nbands > 256 || (nfft > 0 && !fft) ||
        (nwave > 0 && !wave) || (nbands > 0 && !bands))
        return -3;
    const float k3 = gl_c01(P[P_K3]);
    int rc = gl_begin(dst, dW, dH, dpitch, down, sh_r, sh_g, sh_b, acc, W, H, bloom, BW, BH, k3 > 0.0f);
    if (rc) return rc;
    const float dt = gl_clampf(P[P_DT], 0.0f, 0.1f);
    G.t += dt;
    if (G.t > 1.0e5f) G.t = 0.0f;
    const float level = gl_c01(P[P_LEVEL]), bass = gl_c01(P[P_BASS]), treble = gl_c01(P[P_TREBLE]);
    const float volt = gl_c01(P[P_K1]);
    G.fizz_t += dt * (0.6f + 1.8f * treble + 0.8f * volt);
    if (G.fizz_t > 1.0e4f) G.fizz_t = 0.0f;
    if (P[P_KICK] > 0.5f) G.kick_glow = 1.0f;
    G.kick_glow = fmaxf(0.0f, G.kick_glow - dt * 3.0f);
    if (P[P_TRIG] > 0.5f || P[P_HOLD] > 0.5f) G.touch = 1.0f;
    G.touch = fmaxf(0.0f, G.touch - dt * 1.2f);            /* let go: the hand lifts within ~0.8 s */
    /* the globe turns on knob 2 */
    float k2 = (gl_c01(P[P_K2]) - 0.5f) * 2.0f;
    k2 = fabsf(k2) < 0.08f ? 0.0f : copysignf((fabsf(k2) - 0.08f) / 0.92f, k2);
    G.yaw += k2 * 0.6f * dt;
    if (G.yaw > 1.0e4f || G.yaw < -1.0e4f) G.yaw = 0.0f;
    gl_camera(&g_cam, G.yaw, 0.18f + 0.06f * gl_sin(G.t * 0.13f), 9.0f, 0.85f, 0.0f, 0.0f, 0.0f);

    float fg[3] = { gl_c01(P[P_FG_R]), gl_c01(P[P_FG_G]), gl_c01(P[P_FG_B]) };
    if (fg[0] + fg[1] + fg[2] < 0.15f) { fg[0] = 0.75f; fg[1] = 0.35f; fg[2] = 1.0f; }   /* plasma violet */
    const float R = 3.0f;
    /* how many filaments burn: the loudness and the voltage */
    float want = (2.0f + 10.0f * level + 7.0f * volt) * (1.0f + 0.4f * G.kick_glow);
    for (int i = 0; i < NFIL; i++) {
        Fil *f = &G.f[i];
        float e = 0.0f;
        if (bands && nbands >= 2 * NFIL) e = 0.5f * (gl_c01(bands[2 * i]) + gl_c01(bands[2 * i + 1]));
        else e = level * (0.6f + 0.4f * gl_sin(G.t * (0.9f + 0.31f * (float)i)));
        f->lvl += (e - f->lvl) * fminf(1.0f, dt * 10.0f);
        /* a filament burns when there is room for it (the first `want`, by index, fading), the louder ones longer */
        float target = (float)i < want ? 1.0f : 0.0f;
        f->on += (target - f->on) * fminf(1.0f, dt * (target > f->on ? 8.0f : 3.0f));
        /* the meeting point drifts over the glass, drawn up a little (hot plasma rises), wandering more at high voltage */
        f->vth += (gl_rnd() - 0.5f) * dt * (0.8f + 2.0f * volt) - 0.12f * dt * (f->th - 1.1f);
        f->vph += (gl_rnd() - 0.5f) * dt * (1.2f + 3.0f * volt);
        f->vth *= expf(-dt * 0.8f);
        f->vph *= expf(-dt * 0.6f);
    }
    /* filaments repel each other (like charges): the burning ones spread out over the glass */
    for (int i = 0; i < NFIL; i++) {
        Fil *f = &G.f[i];
        if (f->on < 0.05f) continue;
        float pi[3];
        sph(f->th, f->ph, 1.0f, pi);
        float fx = 0.0f, fy = 0.0f, fz = 0.0f;
        for (int j = 0; j < NFIL; j++) {
            if (j == i || G.f[j].on < 0.05f) continue;
            float pj[3];
            sph(G.f[j].th, G.f[j].ph, 1.0f, pj);
            float dx = pi[0] - pj[0], dy = pi[1] - pj[1], dz = pi[2] - pj[2];
            float d2 = dx * dx + dy * dy + dz * dz + 0.02f;
            float k = G.f[j].on / (d2 * d2);
            fx += dx * k; fy += dy * k; fz += dz * k;
        }
        /* the push, in the angles' directions: d(theta) along (cos th cos ph, -sin th, cos th sin ph), d(phi) along
         * (-sin ph, 0, cos ph) */
        float st = sinf(f->th), ct = cosf(f->th), sp = sinf(f->ph), cp = cosf(f->ph);
        float gth = fx * ct * cp - fy * st + fz * ct * sp, gph = -fx * sp + fz * cp;
        f->vth += gl_clampf(gth, -20.0f, 20.0f) * 0.004f * dt * 60.0f * 0.25f;
        f->vph += gl_clampf(gph, -20.0f, 20.0f) * 0.004f * dt * 60.0f * 0.25f / fmaxf(st, 0.2f);
    }
    for (int i = 0; i < NFIL; i++) {
        Fil *f = &G.f[i];
        f->vth = gl_clampf(f->vth, -1.5f, 1.5f);
        f->vph = gl_clampf(f->vph, -2.5f, 2.5f);
        f->th = gl_clampf(f->th + f->vth * dt, 0.15f, 2.99f);
        f->ph += f->vph * dt;
        if (f->ph > 1.0e3f || f->ph < -1.0e3f) f->ph = 0.0f;
    }
    /* the glass: its rim (the circle facing the camera), brighter at the upper left */
    {
        const int NR = 64;
        float prev[3] = { 0, 0, 0 };
        for (int i = 0; i <= NR; i++) {
            float a = (float)i / NR * 6.2831853f;
            /* a circle about the view axis: rotate (cos a, sin a, 0) in camera space back to the world */
            float cx = cosf(a) * R, cyv = sinf(a) * R;
            /* camera space x = world x1 (after yaw), y = y2: undo pitch (y2 = cp y - sp z1, with z1 chosen 0 plane
             * facing the camera: the sphere's outline is where the view grazes it - near enough the circle through
             * the centre, radius R) */
            float y = cyv * g_cam.cp, z1 = -cyv * g_cam.sp;
            float x = g_cam.cy * cx + g_cam.sy * z1, z = -g_cam.sy * cx + g_cam.cy * z1;
            if (i > 0) {
                float hl = 0.5f + 0.5f * cosf(a - 2.3f);       /* brighter upper left */
                float b = 0.07f + 0.10f * hl * hl * hl;
                float c[3] = { 0.6f * b + fg[0] * 0.05f, 0.7f * b + fg[1] * 0.05f, 0.9f * b + fg[2] * 0.05f };
                gl_line3(&g_cam, prev[0], prev[1], prev[2], x, y, z, 0.03f, c, c);
            }
            prev[0] = x; prev[1] = y; prev[2] = z;
        }
    }
    /* the electrode */
    {
        float g = 0.9f + 1.4f * bass + 1.2f * G.kick_glow;
        gl_sprite3(&g_cam, 0.0f, 0.0f, 0.0f, 0.55f + 0.2f * bass, fg[0] * g + 0.5f, fg[1] * g + 0.5f, fg[2] * g + 0.5f);
        gl_sprite3(&g_cam, 0.0f, 0.0f, 0.0f, 1.5f, fg[0] * 0.25f * g, fg[1] * 0.25f * g, fg[2] * 0.25f * g);
    }
    /* the touch point: toward the viewer, up and to the right of the middle (straight on, the gathered filaments
     * pointed at the camera and showed as a knot in the middle) */
    float tp[3];
    {
        /* camera space (0.5 R, 0.35 R, 0.79 R) back to the world (the rim's inverse: undo the pitch, then the yaw) */
        float x2 = R * 0.5f, y2 = R * 0.35f, z2 = R * 0.79f;
        float z1 = -g_cam.sp * y2 + g_cam.cp * z2;
        tp[0] = g_cam.cy * x2 + g_cam.sy * z1;
        tp[1] = g_cam.cp * y2 + g_cam.sp * z2;
        tp[2] = -g_cam.sy * x2 + g_cam.cy * z1;
        float n = sqrtf(tp[0] * tp[0] + tp[1] * tp[1] + tp[2] * tp[2]);
        for (int k = 0; k < 3; k++) tp[k] *= R / n;
    }
    const float pull = G.touch > 0.0f ? gl_c01(G.touch * 1.6f) : 0.0f;
    /* the filaments */
    const float ws = 0.018f + 0.012f * bass;
    for (int i = 0; i < NFIL; i++) {
        Fil *f = &G.f[i];
        if (f->on < 0.02f) continue;
        float end[3];
        sph(f->th, f->ph, R * 0.985f, end);
        if (pull > 0.0f) {                                    /* a hand on the glass: they gather to it */
            float s = pull * pull * (3.0f - 2.0f * pull);
            for (int k = 0; k < 3; k++) end[k] += (tp[k] - end[k]) * s;
            float n = sqrtf(end[0] * end[0] + end[1] * end[1] + end[2] * end[2]) + 1e-6f;
            for (int k = 0; k < 3; k++) end[k] *= R * 0.985f / n;
        }
        const float a[3] = { end[0] * 0.13f, end[1] * 0.13f, end[2] * 0.13f };   /* from the electrode's surface */
        /* a sideways direction: across the line to the glass */
        float up[3] = { 0.0f, 1.0f, 0.0f };
        if (fabsf(end[1]) > R * 0.9f) { up[0] = 1.0f; up[1] = 0.0f; }
        float side[3] = { end[1] * up[2] - end[2] * up[1], end[2] * up[0] - end[0] * up[2], end[0] * up[1] - end[1] * up[0] };
        float sn = sqrtf(side[0] * side[0] + side[1] * side[1] + side[2] * side[2]) + 1e-6f;
        for (int k = 0; k < 3; k++) side[k] /= sn;
        float bright = f->on * (0.35f + 1.3f * f->lvl) * (1.0f + 0.9f * G.kick_glow + 0.8f * pull);
        float bow = 0.35f * gl_sin(G.t * 0.6f + (float)i * 1.3f) + 0.15f;
        float fz = 0.10f + 0.28f * treble + 0.18f * volt;
        float col[3] = { fg[0] * 0.85f + 0.15f, fg[1] * 0.85f + 0.15f, fg[2] * 0.85f + 0.15f };
        strand(a, end, side, bow, fz, f->seed, col, bright, ws, NSEG);
        /* a few forks near the glass */
        int nb = 1 + (int)(volt * 2.5f + f->lvl * 1.5f);
        for (int b = 0; b < nb && b < 4; b++) {
            float s0 = 0.62f + 0.1f * (float)b;
            float st[3], en[3];
            for (int k = 0; k < 3; k++) st[k] = a[k] + (end[k] - a[k]) * s0;
            float ang = gl_noise1(G.t * 0.7f + (float)b * 3.1f, f->seed + (uint32_t)b * 77u) * 0.5f;
            for (int k = 0; k < 3; k++) en[k] = end[k] + side[k] * ang * R * 0.5f;
            float n = sqrtf(en[0] * en[0] + en[1] * en[1] + en[2] * en[2]) + 1e-6f;
            for (int k = 0; k < 3; k++) en[k] *= R * 0.985f / n;
            strand(st, en, side, 0.08f, fz * 0.6f, f->seed ^ (uint32_t)(b * 2654435761u), col, bright * 0.45f, ws * 0.7f, 8);
            gl_sprite3(&g_cam, en[0], en[1], en[2], 0.12f, col[0] * bright * 0.6f, col[1] * bright * 0.6f, col[2] * bright * 0.6f);
        }
        /* where it meets the glass: a hot spot */
        float hs = bright * (1.0f + 0.5f * f->lvl);
        gl_sprite3(&g_cam, end[0], end[1], end[2], 0.22f + 0.1f * f->lvl, col[0] * hs + 0.3f * hs, col[1] * hs + 0.3f * hs,
                   col[2] * hs + 0.3f * hs);
    }
    /* knob 3: bloom up to the middle, a dead band, then trails from 0.6 */
    const float bloom_amt = k3 < 0.5f ? k3 * 2.0f * 1.7f : 1.7f;
    const float decay = k3 >= 0.6f ? 0.40f + (k3 - 0.6f) * (0.50f / 0.4f) : 0.0f;
    const float bg[3] = { gl_c01(P[P_BG_R]), gl_c01(P[P_BG_G]), gl_c01(P[P_BG_B]) };
    gl_end(bloom_amt, decay, 1.0f, bg, P[P_PREFILLED] > 0.5f);
    return 0;
}

/* for tests and DEBUG: glow's stats, then the filaments burning, the touch, the first filament's level */
void fx_stats(float *out, int n) {
    float v[11];
    gl_stats(v);
    int on = 0;
    for (int i = 0; i < NFIL; i++) on += G.f[i].on > 0.5f;
    v[8] = (float)on;
    v[9] = G.touch;
    v[10] = G.f[0].lvl;
    for (int i = 0; i < n && i < 11; i++) out[i] = v[i];
}
