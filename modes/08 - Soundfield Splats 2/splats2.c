/* splats2.c - the kernel of the EYESY mode "S - Soundfield Splats 2": sound, slowed ~1000x, moving through a disc
 * of air in 3D, drawn with glow.h (the Swarm Visualizer's LEDs, bloom and trails).
 *
 * Physics (as S - Soundfield Splats, v1): every source radiates the same signal outward at a "speed of sound"; an
 * air particle at distance r from a source feels that source's signal from r / c seconds ago, softened by
 * r0 / (r + r0) (spherical spreading), and is pushed along the direction of travel by it (air moves along the wave);
 * the pressures add, so several sources interfere. v2 also lifts each particle by its pressure, so the disc becomes a
 * rippling surface, and the sources slide between arrangements (knob 2): one in the middle, a pair, a triangle, a
 * square, a ring of six - each a different interference figure.
 * The signal (made here, one sample every 1/120 s, so it does not depend on the frame rate): a sine at a slowed
 * "pitch" (0.3 .. 1.4 Hz, from the music's spectral centroid, or the waveform's zero crossings without a spectrum),
 * its loudness from an auto-gained envelope, plus a bipolar pulse (a crest, then a trough) on every kick, and a
 * sharp one on Trigger.
 * Compressions glow in the foreground colour, rarefactions in its complement; particles out of the focal plane
 * spread and dim (depth of field, the light conserved). */
#include "glow.h"

#define SF_VERSION 1
#define MAXP 8192
#define NSLOT 6                    /* source slots: layouts put several on one spot, with the weights split */
#define HZ 120.0f                  /* the signal's sample rate */
#define HLEN 2048                  /* ~17 s of it: waves take up to ~14 s to cross the disc at the slowest */
#define CLOUD_R 1.5f

enum {
    P_DT, P_K1, P_K2, P_K3, P_K4, P_K5,
    P_FG_R, P_FG_G, P_FG_B, P_BG_R, P_BG_G, P_BG_B,
    P_LEVEL, P_BASS, P_MID, P_TREBLE, P_BEAT, P_KICK, P_TRIG, P_PREFILLED,
    P_WAVE_HZ,                     /* the waveform's sample rate (32000 on stereopsis, 2000 from the stock scope) */
    P_SIZE,                        /* particle size x (1) */
    P_COUNT
};

/* the particle pass's inputs (set each frame), and its outputs per particle: the sprite (x, y, radius, light), or a
 * radius of 0 = not drawn */
typedef struct {
    int ns;
    float src[NSLOT * 3], sw[NSLOT];
    float spu_s, r0, sat, dispg, lift, focus, aper, glowb, size, res;
    float cpos[3], cneg[3], cbase[3];
    GCam cam;
} Pass;

static Pass g_pass;
static float g_out[MAXP * 6];

static struct {
    int ready, n;
    float pos[MAXP * 3];           /* rest positions */
    float seed[MAXP];
    float hist[HLEN];              /* the signal, newest at hist[head] */
    int head;
    float t, t_sig;                /* time; the signal's last sample time */
    float phase, env, peak, slow, refr, pitch_vis;
    float pend[32];                /* pulse samples still to come (at HZ) */
    float yaw, sw[NSLOT];          /* camera yaw; the slots' current weights */
    float sp[NSLOT * 3];           /* the slots' current positions (eased to the layout) */
    float kick_glow;
} S;

int fx_version(void) { return SF_VERSION; }
int fx_param_count(void) { return P_COUNT; }
int fx_ready(void) { return S.ready; }
int fx_threads(int n) { return gl_set_threads(n); }
void fx_trails_reset(void) { gl_trails_reset(); }
float fx_bench(int which, int n, float r) { return gl_bench(which, n, r); }

/* about n particles of air on a hexagonal mesh filling the disc (v1 scattered them at random: waves read as noise
 * there; on a mesh every compression is a clean band and interference draws its fringes), a little jitter so the
 * mesh does not beat with the pixel grid */
int fx_init(int n, int seed) {
    if (n < 64 || n > MAXP) return -1;
    memset(&S, 0, sizeof S);
    gl_rng_state = 1234u + (uint32_t)seed * 2654435761u;
    if (!gl_rng_state) gl_rng_state = 1u;
    const float area = 3.14159265f * CLOUD_R * CLOUD_R;
    float a = sqrtf(area / ((float)n * 0.8660254f));          /* the mesh spacing for ~n points */
    int i = 0;
    for (int tries = 0; tries < 8; tries++) {                 /* shrink the spacing until n fit */
        i = 0;
        int rows = (int)(2.0f * CLOUD_R / (a * 0.8660254f)) + 2;
        for (int r = -rows / 2; r <= rows / 2 && i < n; r++)
            for (int c = -(int)(CLOUD_R / a) - 2; c <= (int)(CLOUD_R / a) + 2 && i < n; c++) {
                float x = ((float)c + ((r & 1) ? 0.5f : 0.0f)) * a, z = (float)r * a * 0.8660254f;
                if (x * x + z * z > CLOUD_R * CLOUD_R) continue;
                S.pos[i * 3] = x + (gl_rnd() - 0.5f) * 0.12f * a;
                S.pos[i * 3 + 1] = (gl_rnd() - 0.5f) * 0.02f;
                S.pos[i * 3 + 2] = z + (gl_rnd() - 0.5f) * 0.12f * a;
                S.seed[i] = gl_rnd();
                i++;
            }
        if (i >= (n * 97) / 100) break;
        a *= 0.99f;
    }
    S.n = i;
    S.peak = 0.02f;
    S.yaw = 0.6f;
    S.pitch_vis = 0.6f;
    for (int k = 0; k < NSLOT; k++) S.sw[k] = 1.0f / NSLOT;
    S.ready = 1;
    return 0;
}

/* source arrangements, each as NSLOT slots (x, z, weight); a layout's weights sum to 1 */
#define SEP 0.42f
static void layout(int which, float out[NSLOT * 3]) {
    static const float TRI = 0.48f, SQ = 0.40f, HEX = 0.55f;
    for (int k = 0; k < NSLOT; k++) {
        float x = 0.0f, z = 0.0f, w = 1.0f / NSLOT;
        switch (which) {
        case 0:                                              /* one, in the middle */
            break;
        case 1:                                              /* a pair (v1's left and right) */
            x = (k & 1) ? SEP : -SEP;
            break;
        case 2: {                                            /* a triangle */
            float a = (float)(k % 3) * 2.0943951f - 1.5707963f;
            x = cosf(a) * TRI; z = sinf(a) * TRI;
            break;
        }
        case 3:                                              /* a square: four corners, two slots silent */
            if (k < 4) {
                float a = (float)k * 1.5707963f + 0.7853982f;
                x = cosf(a) * SQ; z = sinf(a) * SQ; w = 0.25f;
            } else {
                x = 0.0f; z = 0.0f; w = 0.0f;
            }
            break;
        default: {                                           /* a ring of six */
            float a = (float)k * 1.0471976f;
            x = cosf(a) * HEX; z = sinf(a) * HEX;
            break;
        }
        }
        out[k * 3] = x;
        out[k * 3 + 1] = z;
        out[k * 3 + 2] = w;
    }
}

static float hist_at(float age_s) {                          /* the signal age_s seconds ago, lerped */
    float k = age_s * HZ;
    if (!(k >= 0.0f)) k = 0.0f;
    if (!(k < (float)(HLEN - 2))) return 0.0f;
    int i = (int)k;
    float f = k - (float)i;
    int a = (S.head - i) & (HLEN - 1), b = (S.head - i - 1) & (HLEN - 1);
    return S.hist[a] * (1.0f - f) + S.hist[b] * f;
}

/* PULSE: crest then trough, per 1/30 s (v1's shape), summing to zero - spread over the 1/120 s samples */
static const float PULSE[6] = { 1.0f, 0.5f, -0.3f, -0.6f, -0.4f, -0.2f };

static void add_pulse(float amp) {
    for (int i = 0; i < 24; i++) S.pend[i] += amp * PULSE[i / 4];
}

/* the signal up to time now: one sample per 1/120 s */
static void signal(float dt, float level, int kick, int trig, float centroid_hz) {
    /* automatic gain: normalise the level against a peak tracker (instant attack, ~6 s release) over a floor */
    float pk = fmaxf(fmaxf(level, S.peak * (1.0f - 0.17f * dt)), 0.02f);
    S.peak = pk;
    float lv = fminf(1.0f, level / pk);
    S.env += (lv - S.env) * (lv > S.env ? 0.55f : 0.10f) * fminf(1.0f, dt * 30.0f);
    /* onsets: the level jumping above its slow average (no analyser), or the analyser's kicks */
    S.slow += (lv - S.slow) * fminf(1.0f, 2.5f * dt);
    S.refr = fmaxf(0.0f, S.refr - dt);
    float hit = fmaxf(0.0f, lv - S.slow * 1.4f - 0.12f);
    float amp = kick ? 1.0f : fminf(1.2f, hit * 2.5f);
    if (trig) { add_pulse(1.4f); S.refr = 0.25f; }
    else if (amp > 0.05f && S.refr <= 0.0f) { add_pulse(amp); S.refr = 0.25f; }
    /* the slowed pitch: 80 Hz -> 0.3 Hz .. 1 kHz -> 1.4 Hz, on a log scale */
    if (centroid_hz > 0.0f) {
        float u = gl_c01(logf(fmaxf(centroid_hz, 1.0f) / 80.0f) / logf(1000.0f / 80.0f));
        S.pitch_vis += (0.3f + 1.1f * u - S.pitch_vis) * fminf(1.0f, dt * 3.0f);
    }
    S.t += dt;
    while (S.t_sig + 1.0f / HZ <= S.t) {
        S.t_sig += 1.0f / HZ;
        S.phase += 6.2831853f * S.pitch_vis / HZ;
        if (S.phase > 6.2831853f) S.phase -= 6.2831853f;
        S.head = (S.head + 1) & (HLEN - 1);
        S.hist[S.head] = 0.35f * S.env * sinf(S.phase) + S.pend[0];
        memmove(S.pend, S.pend + 1, sizeof(float) * 31);
        S.pend[31] = 0.0f;
    }
    if (S.t > 1.0e5f) { S.t = 0.0f; S.t_sig = 0.0f; }
}

/* the spectrum's centroid, Hz (0 without one) / the waveform's zero-crossing pitch */
static float centroid(const float *fft, int nfft, float fft_hz, const float *wave, int nwave, float wave_hz) {
    if (fft && nfft > 8 && fft_hz > 0.0f) {
        double s = 0.0, sw = 0.0;
        int top = (int)(4000.0f / fft_hz);
        top = top < nfft ? top : nfft;
        for (int k = 1; k < top; k++) {
            float v = gl_c01(fft[k]);
            v *= v;
            s += v;
            sw += v * (double)k;
        }
        return s > 1e-6 ? (float)(sw / s) * fft_hz : 0.0f;
    }
    if (wave && nwave > 8 && wave_hz > 0.0f) {
        int zc = 0;
        for (int i = 1; i < nwave; i++)
            zc += (wave[i] >= 0.0f) != (wave[i - 1] >= 0.0f);
        return (float)zc / (float)(nwave - 1) * wave_hz * 0.5f;
    }
    return 0.0f;
}

/* like the Swarm Visualizer's: the colour turned about the grey axis */
static void hue_turn(const float in[3], float turns, float out[3]) {
    const float hk = 0.57735026919f;
    float a = turns * 6.2831853f, hc = cosf(a), hs = sinf(a);
    float dot = hk * (in[0] + in[1] + in[2]);
    float kx[3] = { hk * (in[2] - in[1]), hk * (in[0] - in[2]), hk * (in[1] - in[0]) };
    for (int c = 0; c < 3; c++) out[c] = gl_c01(in[c] * hc + kx[c] * hs + hk * dot * (1.0f - hc));
}

/* particles [i0, i1): pressure and displacement from every source, the lift, colour, projection, depth of field ->
 * g_out (writes only these particles' entries: safe on two threads) */
static void particles(void *ctx, int i0, int i1) {
    const Pass *Q = (const Pass *)ctx;
    const int ns = Q->ns;
    for (int i = i0; i < i1; i++) {
        const float *p = &S.pos[i * 3];
        float *o = &g_out[i * 6];
        float dx = 0.0f, dy = 0.0f, dz = 0.0f, pres = 0.0f;
        for (int j = 0; j < ns; j++) {
            float ex = p[0] - Q->src[j * 3], ey = p[1] - Q->src[j * 3 + 1], ez = p[2] - Q->src[j * 3 + 2];
            float r = sqrtf(ex * ex + ey * ey + ez * ez) + 1e-6f;
            float v = hist_at(r * Q->spu_s) * (Q->r0 / (r + Q->r0)) * Q->sw[j];
            pres += v;
            float k = v / r;
            dx += ex * k; dy += ey * k; dz += ez * k;
        }
        /* the lift, soft-clipped: short waves (slow sound) would throw neighbours far apart */
        float x = p[0] + dx * Q->dispg, y = p[1] + dy * Q->dispg + Q->lift * pres / (1.0f + 2.0f * fabsf(pres)),
              z = p[2] + dz * Q->dispg;
        float tt = gl_c01(fabsf(pres) / Q->sat);
        const float *cw = pres >= 0.0f ? Q->cpos : Q->cneg;
        float cr = Q->cbase[0] + (cw[0] - Q->cbase[0]) * tt, cg = Q->cbase[1] + (cw[1] - Q->cbase[1]) * tt,
              cb = Q->cbase[2] + (cw[2] - Q->cbase[2]) * tt;
        /* crests glow with a squared curve, troughs half as much, air at rest just shows: wavefronts read as rings */
        float bright = (0.30f + 2.6f * tt * tt * (pres >= 0.0f ? 1.0f : 0.5f)) * Q->glowb;
        float sx, sy, d = gl_project(&Q->cam, x, y, z, &sx, &sy);
        if (d <= 0.2f) {
            o[2] = 0.0f;
            continue;
        }
        float r = Q->size * (2.8f + 0.6f * S.seed[i]) * Q->res * (3.6f / d);
        float coc = Q->aper * fabsf(d - Q->focus) / d;       /* depth of field: spread, light kept */
        float re = sqrtf(r * r + coc * coc);
        float k = bright * (r * r) / (re * re);
        o[0] = sx; o[1] = sy; o[2] = re;
        o[3] = cr * k; o[4] = cg * k; o[5] = cb * k;
    }
}

/* the frame. Returns 0, or < 0 if the arguments were rejected (nothing drawn) */
int fx_frame(uint8_t *dst, int dW, int dH, int dpitch, int down, int sh_r, int sh_g, int sh_b, float *acc, int W, int H,
             float *bloom, int BW, int BH, const float *fft, int nfft, float fft_hz, const float *wave, int nwave,
             const float *bands, int nbands, const float *P) {
    (void)bands;
    (void)nbands;
    if (!S.ready) return -9;
    if (!P) return -1;
    if (nfft < 0 || nfft > 65536 || nwave < 0 || nwave > 65536 || (nfft > 0 && !fft) || (nwave > 0 && !wave)) return -3;
    const float k3 = gl_c01(P[P_K3]);
    int rc = gl_begin(dst, dW, dH, dpitch, down, sh_r, sh_g, sh_b, acc, W, H, bloom, BW, BH, k3 > 0.0f);
    if (rc) return rc;
    if (nfft < 2 || !(fft_hz > 0.0f && fft_hz < 1.0e5f)) fft = NULL;
    if (nwave < 2) wave = NULL;
    const float dt = gl_clampf(P[P_DT], 0.0f, 0.1f);
    const float level = gl_c01(P[P_LEVEL]), beat = gl_c01(P[P_BEAT]);
    const int kick = P[P_KICK] > 0.5f, trig = P[P_TRIG] > 0.5f;
    signal(dt, level, kick, trig, centroid(fft, nfft, fft_hz, wave, nwave, gl_clampf(P[P_WAVE_HZ], 0.0f, 1.0e6f)));
    if (kick) S.kick_glow = 1.0f;
    S.kick_glow = fmaxf(0.0f, S.kick_glow - dt * 3.0f);

    /* knob 1: the speed of sound (world units a second; the disc is 1.5 across the middle) */
    const float c_sound = 0.16f + 1.5f * gl_c01(P[P_K1]);
    /* knob 2: the sources - five arrangements along the knob, slid between */
    float lay_a[NSLOT * 3], lay_b[NSLOT * 3];
    float k2 = gl_c01(P[P_K2]) * 4.0f;
    int la = (int)floorf(k2);
    la = la > 3 ? 3 : la;
    float lf = k2 - (float)la;
    lf = lf * lf * (3.0f - 2.0f * lf);
    layout(la, lay_a);
    layout(la + 1, lay_b);
    const float ek = fminf(1.0f, dt * 4.0f);
    for (int k = 0; k < NSLOT; k++) {
        float x = lay_a[k * 3] + (lay_b[k * 3] - lay_a[k * 3]) * lf;
        float z = lay_a[k * 3 + 1] + (lay_b[k * 3 + 1] - lay_a[k * 3 + 1]) * lf;
        float w = lay_a[k * 3 + 2] + (lay_b[k * 3 + 2] - lay_a[k * 3 + 2]) * lf;
        S.sp[k * 3] += (x - S.sp[k * 3]) * ek;
        S.sp[k * 3 + 2] += (z - S.sp[k * 3 + 2]) * ek;
        S.sw[k] += (w - S.sw[k]) * ek;
    }
    /* the sources drift gently, as v1's - by where they are, so slots on one spot stay together (and merge) */
    float src[NSLOT * 3], sw[NSLOT];
    int ns = 0;
    const float wob = 0.10f, t = S.t;
    for (int k = 0; k < NSLOT; k++) {
        if (S.sw[k] < 1e-3f) continue;
        const float bx = S.sp[k * 3], bz = S.sp[k * 3 + 2];
        float x = bx + wob * gl_sin(t * 0.21f + 3.0f * bx + 5.0f * bz);
        float y = wob * gl_sin(t * 0.17f + 4.0f * bx + 2.0f * bz + 1.0f);
        float z = bz + wob * gl_cos(t * 0.13f + 2.0f * bx + 3.0f * bz + 0.5f);
        int merged = 0;                                      /* slots on the same spot: one source, weights added */
        for (int j = 0; j < ns && !merged; j++)
            if (fabsf(src[j * 3] - x) < 1e-3f && fabsf(src[j * 3 + 2] - z) < 1e-3f && fabsf(src[j * 3 + 1] - y) < 1e-3f) {
                sw[j] += S.sw[k];
                merged = 1;
            }
        if (merged) continue;
        src[ns * 3] = x; src[ns * 3 + 1] = y; src[ns * 3 + 2] = z;
        sw[ns] = S.sw[k];
        ns++;
    }
    /* a single source radiates at full strength; several share it by their weights, scaled by sqrt(count) (the
     * crests where they meet in phase still add up past one source's) */
    float wsum = 0.0f;
    for (int j = 0; j < ns; j++) wsum += sw[j];
    for (int j = 0; j < ns; j++) sw[j] = sw[j] / fmaxf(wsum, 1e-3f) * sqrtf((float)ns);

    /* colours: the foreground (knob 4) for compressions, its complement for rarefactions, a dim mix at rest */
    float cpos[3] = { gl_c01(P[P_FG_R]), gl_c01(P[P_FG_G]), gl_c01(P[P_FG_B]) }, cneg[3], cbase[3];
    float grey = fmaxf(cpos[0], fmaxf(cpos[1], cpos[2])) - fminf(cpos[0], fminf(cpos[1], cpos[2]));
    if (grey < 0.08f) {                                      /* greys: a darker / lighter grey instead of a hue */
        float l = (cpos[0] + cpos[1] + cpos[2]) > 1.5f ? 0.35f : 0.75f;
        cneg[0] = cneg[1] = cneg[2] = l;
    } else {
        hue_turn(cpos, 0.47f, cneg);
    }
    for (int c = 0; c < 3; c++) cbase[c] = 0.25f * cpos[c] + 0.08f * cneg[c] + 0.03f;

    /* camera: a slow orbit of its own (kicks nudge it), a breathing tilt */
    S.yaw += (0.07f + 0.35f * S.kick_glow * S.kick_glow) * dt;
    if (S.yaw > 1.0e4f) S.yaw = 0.0f;
    const float pitch = 0.62f + 0.10f * gl_sin(t * 0.07f);
    GCam cam;
    gl_camera(&cam, S.yaw, pitch, 3.6f, 0.56f, 0.0f, 0.0f, 0.0f);      /* v1's framing: focal 1.75 x the height */
    const float res = (float)H / 360.0f;
    const float size = gl_clampf(P[P_SIZE], 0.2f, 5.0f);
    const float spu_s = 1.0f / c_sound;                      /* seconds per world unit */
    const float r0 = 0.9f, sat = 0.45f, dispg = 0.16f, lift = 0.6f;
    const float focus = 3.6f, aper = 3.5f * res;
    const float glowb = 1.0f + 0.35f * beat + 0.25f * S.kick_glow;
    Pass *Q = &g_pass;
    Q->ns = ns;
    memcpy(Q->src, src, sizeof src);
    memcpy(Q->sw, sw, sizeof sw);
    Q->spu_s = spu_s; Q->r0 = r0; Q->sat = sat; Q->dispg = dispg; Q->lift = lift; Q->focus = focus; Q->aper = aper;
    Q->glowb = glowb; Q->size = size; Q->res = res;
    memcpy(Q->cpos, cpos, sizeof cpos);
    memcpy(Q->cneg, cneg, sizeof cneg);
    memcpy(Q->cbase, cbase, sizeof cbase);
    Q->cam = cam;
    gl_parallel_for(particles, Q, S.n, 256);                 /* the physics on both cores ... */
    for (int i = 0; i < S.n; i++) {                          /* ... the sprites in order, here */
        const float *o = &g_out[i * 6];
        if (o[2] > 0.0f) gl_sprite(o[0], o[1], o[2], o[3], o[4], o[5]);
    }
    for (int j = 0; j < ns; j++) {                           /* the sources */
        float sx, sy, d = gl_project(&cam, src[j * 3], src[j * 3 + 1], src[j * 3 + 2], &sx, &sy);
        if (d <= 0.2f) continue;
        float lv = S.env * fminf(1.0f, sw[j] * (float)ns);
        float r = (6.0f + 7.0f * lv) * res * (3.6f / d);
        float b = 1.2f * (0.35f + lv + 0.6f * S.kick_glow);
        gl_sprite(sx, sy, r, cpos[0] * b + 0.2f, cpos[1] * b + 0.2f, cpos[2] * b + 0.2f);       /* the halo */
        gl_sprite(sx, sy, r * 0.35f, 1.5f + b, 1.5f + b, 1.5f + b);                            /* a white-hot core */
    }
    /* knob 3: bloom up to the middle, a dead band, then trails from 0.6 */
    const float bloom_amt = k3 < 0.5f ? k3 * 2.0f * 1.3f : 1.3f;
    const float decay = k3 >= 0.6f ? 0.30f + (k3 - 0.6f) * (0.58f / 0.4f) : 0.0f;
    const float bg[3] = { gl_c01(P[P_BG_R]), gl_c01(P[P_BG_G]), gl_c01(P[P_BG_B]) };
    gl_end(bloom_amt, decay, 1.0f, bg, P[P_PREFILLED] > 0.5f);
    return 0;
}

/* for tests and DEBUG: glow's stats (bin, bloom, final ms; tiles lit, bloom, dark; primitives, dropped), then the
 * signal's envelope, slowed pitch and newest sample */
void fx_stats(float *out, int n) {
    float v[11];
    gl_stats(v);
    v[8] = S.env;
    v[9] = S.pitch_vis;
    v[10] = S.hist[S.head];
    for (int i = 0; i < n && i < 11; i++) out[i] = v[i];
}
