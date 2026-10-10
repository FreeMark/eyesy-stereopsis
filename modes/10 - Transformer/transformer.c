/* transformer.c - the kernel of the EYESY mode "09 - Transformer": a small transformer running on the music, drawn as
 * its weights. With glow.h (the Swarm Visualizer's light).
 *
 * The model: every sixteenth note (at the tempo stereopsis detects; eighth notes of 120 BPM without one) the spectrum
 * becomes a token - the 32 bands paired into 16 and standardised (the shape of the sound, not its loudness). A context
 * of 16 tokens (a bar), one attention layer of 4 heads x 4 dimensions and an MLP (16 -> 48 -> 16), with residuals.
 * Its weights are fixed, from a seed: each head's query and key read mostly its quarter of the spectrum (bass, low
 * mids, high mids, treble), so each head attends to the moments that sounded like now in its range - repetition in
 * the music draws diagonals across the attention maps, a steady beat draws them every 4 tokens. The rest are random
 * matrices with a little structure (a few strong rows and columns). The output is the model's guess at the next
 * token; how wrong the guess turns out to be is its surprise.
 *
 * Drawn as a wall of glass panels the data flows through: EMBED (the last 16 tokens), Q K V, ATTENTION (the four
 * heads' maps), MLP (up and down), OUT (the guess, and the surprise). Each weight is a dot that glows with what it
 * contributes to the newest token right now - w x input, the foreground colour if positive, its complement if
 * negative - so each token sends a wave of light through the panels, along the beams between them.
 * The camera turns (knob 2: up to 75 degrees either way) and comes close (knob 1); close up it glides from panel to
 * panel along the flow.
 * Version 2 (2026-09-30, Free: "the bottom 3 parts dont do anything and the top are twitchy"): the wave ran 55 ms a
 * panel but a token comes every ~120 ms and restarted it, so MLP UP, MLP DOWN and OUT never lit - now it is timed to
 * the token (period / 7 a panel); the Trigger no longer makes a token (stock's loudness trigger made one nearly every
 * frame); every dot eases toward each token's value (~80 ms) instead of jumping; MLP UP's columns follow how active
 * their hidden unit is. */
#include "glow.h"

#define TF_VERSION 2
#define D 16                       /* model width */
#define T 16                       /* context, tokens */
#define NH 4                       /* heads */
#define DH 4                       /* head width */
#define HID 48                     /* MLP hidden */
#define NPANEL 6

enum {
    P_DT, P_K1, P_K2, P_K3, P_K4, P_K5,
    P_FG_R, P_FG_G, P_FG_B, P_BG_R, P_BG_G, P_BG_B,
    P_LEVEL, P_BASS, P_MID, P_TREBLE, P_BEAT, P_KICK, P_TRIG, P_PREFILLED,
    P_WAVE_HZ,
    P_BPM,                         /* stereopsis's tempo (0 = none yet) */
    P_COUNT
};

static struct {
    int ready;
    float Wq[D][D], Wk[D][D], Wv[D][D], Wo[D][D];   /* [out][in] */
    float W1[HID][D], W2[D][HID];
    float X[T][D];                 /* the tokens, oldest first */
    float Kc[T][D], Vc[T][D];      /* their keys and values */
    float A[NH][T][T];             /* attention: row i (a token) over the tokens j <= i */
    float cq[3 * D][D];            /* this token's contributions, w x input (signed), per weight */
    float co[D][D], c1[HID][D], c2[D][HID];
    float sq, so, s1, s2;          /* each matrix's contributions' RMS x 2.5 (a typical dot at ~0.4) */
    float xn[D], att[D], h1[HID], out[D], pred[D];
    float acc[D];                  /* bands gathered toward the next token */
    int acc_n, ntok;
    float t_tok, pass_t, surprise, glow_out;
    float t, yaw, kick_glow, trig_glow, cam_u;
    float wave_dt;                 /* v2: the wave's time per panel (fits a token: period / 7, at most 55 ms) */
    /* v2: what the panels show, easing toward each token's values (normalised by each matrix's RMS) */
    float dX[T][D], dcq[3 * D][D], dA[NH][T][T], dc1[HID][D], dc2[D][HID], dout[D], dact[HID];
} M;

int fx_version(void) { return TF_VERSION; }
int fx_param_count(void) { return P_COUNT; }
int fx_ready(void) { return M.ready; }
int fx_threads(int n) { return gl_set_threads(n); }
void fx_trails_reset(void) { gl_trails_reset(); }
float fx_bench(int which, int n, float r) { return gl_bench(which, n, r); }

static float gauss(void) {                                   /* Box-Muller */
    float u = fmaxf(gl_rnd(), 1e-6f), v = gl_rnd();
    return sqrtf(-2.0f * logf(u)) * cosf(6.2831853f * v);
}

int fx_init(int n, int seed) {
    (void)n;
    memset(&M, 0, sizeof M);
    gl_rng_state = 0x2545F491u ^ ((uint32_t)seed * 2654435761u);
    if (!gl_rng_state) gl_rng_state = 1u;
    /* queries and keys: head h reads its quarter of the spectrum (dims 4h .. 4h+3), near the identity */
    for (int i = 0; i < D; i++)
        for (int j = 0; j < D; j++) {
            int h = i / DH, own = j / DH == h;
            float base = own && (i % DH) == (j % DH) ? 1.0f : 0.0f;
            M.Wq[i][j] = base + (own ? 0.25f : 0.07f) * gauss();
            M.Wk[i][j] = base + (own ? 0.25f : 0.07f) * gauss();
            M.Wv[i][j] = gauss() * 0.35f;
            M.Wo[i][j] = gauss() * 0.3f;
        }
    for (int i = 0; i < HID; i++) {
        float row = (i % 11 == 3) ? 2.5f : 1.0f;             /* a few strong rows: structure in the MLP */
        for (int j = 0; j < D; j++) M.W1[i][j] = gauss() * 0.3f * row * ((j % 7 == 2) ? 1.8f : 1.0f);
    }
    for (int i = 0; i < D; i++)
        for (int j = 0; j < HID; j++) M.W2[i][j] = gauss() * 0.18f;
    M.yaw = 0.0f;
    M.ready = 1;
    return 0;
}

static float gelu(float x) { return 0.5f * x * (1.0f + tanhf(0.7978846f * (x + 0.044715f * x * x * x))); }

/* a new token: the model runs once */
static void token(void) {
    float x[D];
    float mean = 0.0f, var = 0.0f;
    for (int d = 0; d < D; d++) {
        x[d] = M.acc_n ? M.acc[d] / (float)M.acc_n : 0.0f;
        mean += x[d];
    }
    mean /= (float)D;
    for (int d = 0; d < D; d++) var += (x[d] - mean) * (x[d] - mean);
    float sd = sqrtf(var / (float)D) + 0.04f;
    for (int d = 0; d < D; d++) x[d] = (x[d] - mean) / sd * 0.6f;
    /* surprise: how far the last guess was from this */
    float err = 0.0f;
    for (int d = 0; d < D; d++) err += (x[d] - M.pred[d]) * (x[d] - M.pred[d]);
    M.surprise = M.ntok > 2 ? gl_c01(sqrtf(err / (float)D) * 0.8f) : 0.0f;
    /* the context moves one token on */
    memmove(M.X[0], M.X[1], sizeof(float) * D * (T - 1));
    memmove(M.Kc[0], M.Kc[1], sizeof(float) * D * (T - 1));
    memmove(M.Vc[0], M.Vc[1], sizeof(float) * D * (T - 1));
    for (int h = 0; h < NH; h++)
        for (int i = 0; i + 1 < T; i++)
            for (int j = 0; j + 1 < T; j++) M.A[h][i][j] = M.A[h][i + 1][j + 1];
    memcpy(M.X[T - 1], x, sizeof x);
    memcpy(M.xn, x, sizeof x);
    /* q, k, v - and each weight's contribution */
    float q[D], k[D], v[D];
    M.sq = 0.0f;
    for (int i = 0; i < D; i++) {
        float sq = 0.0f, sk = 0.0f, sv = 0.0f;
        for (int j = 0; j < D; j++) {
            float a = M.Wq[i][j] * x[j], b = M.Wk[i][j] * x[j], c = M.Wv[i][j] * x[j];
            M.cq[i][j] = a; M.cq[D + i][j] = b; M.cq[2 * D + i][j] = c;
            M.sq += a * a + b * b + c * c;
            sq += a; sk += b; sv += c;
        }
        q[i] = sq; k[i] = sk; v[i] = sv;
    }
    M.sq = 2.5f * sqrtf(M.sq / (float)(3 * D * D)) + 1e-6f;
    memcpy(M.Kc[T - 1], k, sizeof k);
    memcpy(M.Vc[T - 1], v, sizeof v);
    /* attention for the new token, each head over its keys, causal */
    float heads[D];
    int avail = M.ntok + 1 < T ? M.ntok + 1 : T;
    for (int h = 0; h < NH; h++) {
        float sc[T], mx = -1e30f;
        for (int j = T - avail; j < T; j++) {
            float s = 0.0f;
            for (int d = 0; d < DH; d++) s += q[h * DH + d] * M.Kc[j][h * DH + d];
            /* each head has a habit, as heads in trained models do (a position bias, ALiBi-like): head 0 looks at the
             * last few tokens, head 1 on the beat (every 4 back), head 2 every half bar (8), head 3 by content alone */
            int back = T - 1 - j;
            float bias = h == 0 ? -0.45f * (float)back : h == 1 ? (back % 4 == 0 ? 1.6f : 0.0f)
                       : h == 2 ? (back % 8 == 0 ? 1.8f : 0.0f) : 0.0f;
            sc[j] = s * 2.4f + bias;                         /* content (/ sqrt(4), sharpened) + habit */
            mx = fmaxf(mx, sc[j]);
        }
        float sum = 0.0f;
        for (int j = 0; j < T; j++) {
            float e = j >= T - avail ? expf(sc[j] - mx) : 0.0f;
            M.A[h][T - 1][j] = e;
            sum += e;
        }
        for (int j = 0; j < T; j++) M.A[h][T - 1][j] /= fmaxf(sum, 1e-9f);
        for (int d = 0; d < DH; d++) {
            float s = 0.0f;
            for (int j = 0; j < T; j++) s += M.A[h][T - 1][j] * M.Vc[j][h * DH + d];
            heads[h * DH + d] = s;
        }
    }
    M.so = 0.0f;
    float r[D];
    for (int i = 0; i < D; i++) {
        float s = 0.0f;
        for (int j = 0; j < D; j++) {
            float a = M.Wo[i][j] * heads[j];
            M.co[i][j] = a;
            M.so += a * a;
            s += a;
        }
        M.att[i] = s;
        r[i] = x[i] + s;
    }
    M.so = 2.5f * sqrtf(M.so / (float)(D * D)) + 1e-6f;
    M.s1 = 0.0f;
    for (int i = 0; i < HID; i++) {
        float s = 0.0f;
        for (int j = 0; j < D; j++) {
            float a = M.W1[i][j] * r[j];
            M.c1[i][j] = a;
            M.s1 += a * a;
            s += a;
        }
        M.h1[i] = gelu(s);
    }
    M.s1 = 2.5f * sqrtf(M.s1 / (float)(HID * D)) + 1e-6f;
    M.s2 = 0.0f;
    for (int i = 0; i < D; i++) {
        float s = 0.0f;
        for (int j = 0; j < HID; j++) {
            float a = M.W2[i][j] * M.h1[j];
            M.c2[i][j] = a;
            M.s2 += a * a;
            s += a;
        }
        M.out[i] = r[i] + s;
        M.pred[i] = M.out[i];
    }
    M.s2 = 2.5f * sqrtf(M.s2 / (float)(D * HID)) + 1e-6f;
    M.ntok++;
    M.pass_t = 0.0f;
    memset(M.acc, 0, sizeof M.acc);
    M.acc_n = 0;
}

/* ---------------------------------------------------------------------------------------- drawing */
static GCam g_cam;
static float g_pos[3], g_neg[3];   /* positive / negative contribution colours */

typedef struct { float cx, cy, cz, w, h; const char *label; } Panel;
/* the wall: data flows left to right along the top, down, then right to left along the bottom */
static const Panel PANELS[NPANEL] = {
    { -15.0f, 6.5f, 1.2f, 8.0f, 8.0f, "EMBED" },
    { 0.0f, 6.5f, 0.4f, 13.0f, 5.0f, "Q K V" },
    { 15.5f, 5.0f, -0.4f, 11.0f, 11.0f, "ATTENTION" },
    { 15.5f, -8.5f, -1.2f, 11.0f, 5.0f, "MLP UP" },
    { 0.0f, -8.5f, -0.4f, 13.0f, 5.0f, "MLP DOWN" },
    { -15.0f, -8.5f, 0.4f, 8.0f, 5.0f, "OUT" },
};

/* how lit panel p is now: a wave through the panels after each token (a panel every wave_dt), fading */
static float pass_wave(int p) {
    float t = M.pass_t - M.wave_dt * (float)p;
    if (t < 0.0f) return 0.0f;
    return expf(-t * 5.0f);
}

/* a dot at (u, v) in panel p's own coordinates (0..1 across, 0..1 up), light by a signed contribution c (-1..1) */
static void dot(int p, float u, float v, float c, float base, float r) {
    const Panel *P = &PANELS[p];
    if (!(c == c)) c = 0.0f;                                  /* NaN: dark */
    float x = P->cx + (u - 0.5f) * P->w, y = P->cy + (v - 0.5f) * P->h, z = P->cz;
    float a = fminf(fabsf(c), 1.5f), b = base * 1.4f + 1.6f * powf(a, 0.8f);
    const float *col = c >= 0.0f ? g_pos : g_neg;
    if (b < 0.02f) return;
    gl_sprite3(&g_cam, x, y, z, r, col[0] * b + 0.02f, col[1] * b + 0.02f, col[2] * b + 0.02f);
}

static void panel_frame(int p, float lit) {
    const Panel *P = &PANELS[p];
    float x0 = P->cx - P->w * 0.5f - 0.4f, x1 = P->cx + P->w * 0.5f + 0.4f;
    float y0 = P->cy - P->h * 0.5f - 0.4f, y1 = P->cy + P->h * 0.5f + 0.4f, z = P->cz;
    float b = 0.22f + 0.3f * lit;
    float c[3] = { 0.55f * b + g_pos[0] * 0.3f * lit, 0.65f * b + g_pos[1] * 0.3f * lit, 0.8f * b + g_pos[2] * 0.3f * lit };
    gl_line3(&g_cam, x0, y0, z, x1, y0, z, 0.03f, c, c);
    gl_line3(&g_cam, x1, y0, z, x1, y1, z, 0.03f, c, c);
    gl_line3(&g_cam, x1, y1, z, x0, y1, z, 0.03f, c, c);
    gl_line3(&g_cam, x0, y1, z, x0, y0, z, 0.03f, c, c);
    const float o[3] = { x0, y1 + 0.5f, z }, ax[3] = { 1.0f, 0.0f, 0.0f }, ay[3] = { 0.0f, 1.0f, 0.0f };
    float lc[3] = { c[0] * 1.2f, c[1] * 1.2f, c[2] * 1.2f };
    gl_text3(&g_cam, P->label, o, ax, ay, 1.0f, 0.06f, lc);
}

/* a beam from panel a's edge to panel b's, with the token's pulse running along it */
static void beam(int a, int b, int from_side, int to_side) {
    const Panel *A = &PANELS[a], *B2 = &PANELS[b];
    /* sides: 0 right, 1 down, 2 left */
    float ax = A->cx + (from_side == 0 ? A->w * 0.5f + 0.5f : (from_side == 2 ? -A->w * 0.5f - 0.5f : 0.0f));
    float ay = A->cy + (from_side == 1 ? -A->h * 0.5f - 0.5f : 0.0f);
    float bx = B2->cx + (to_side == 0 ? B2->w * 0.5f + 0.5f : (to_side == 2 ? -B2->w * 0.5f - 0.5f : 0.0f));
    float by = B2->cy + (to_side == 1 ? B2->h * 0.5f + 0.5f : 0.0f);
    float dim[3] = { g_pos[0] * 0.10f + 0.03f, g_pos[1] * 0.10f + 0.03f, g_pos[2] * 0.10f + 0.03f };
    gl_line3(&g_cam, ax, ay, A->cz, bx, by, B2->cz, 0.05f, dim, dim);
    float t = (M.pass_t - M.wave_dt * (float)a) / M.wave_dt; /* the pulse crosses the gap as the wave does */
    if (t > 0.0f && t < 1.0f) {
        float px = ax + (bx - ax) * t, py = ay + (by - ay) * t, pz = A->cz + (B2->cz - A->cz) * t;
        float g = 1.6f + 1.2f * M.kick_glow;
        gl_sprite3(&g_cam, px, py, pz, 0.45f, g_pos[0] * g + 0.4f, g_pos[1] * g + 0.4f, g_pos[2] * g + 0.4f);
    }
}

int fx_frame(uint8_t *dst, int dW, int dH, int dpitch, int down, int sh_r, int sh_g, int sh_b, float *acc, int W, int H,
             float *bloom, int BW, int BH, const float *fft, int nfft, float fft_hz, const float *wave_, int nwave,
             const float *bands, int nbands, const float *P) {
    (void)fft_hz;
    if (!M.ready) return -9;
    if (!P) return -1;
    if (nfft < 0 || nfft > 65536 || nwave < 0 || nwave > 65536 || nbands < 0 || nbands > 256 || (nfft > 0 && !fft) ||
        (nwave > 0 && !wave_) || (nbands > 0 && !bands))
        return -3;
    const float k3 = gl_c01(P[P_K3]);
    int rc = gl_begin(dst, dW, dH, dpitch, down, sh_r, sh_g, sh_b, acc, W, H, bloom, BW, BH, k3 > 0.0f);
    if (rc) return rc;
    const float dt = gl_clampf(P[P_DT], 0.0f, 0.1f);
    M.t += dt;
    if (M.t > 1.0e5f) M.t = 0.0f;
    /* the bands, paired into the model's 16 dimensions (without the analyser: the level, shaped) */
    const float level = gl_c01(P[P_LEVEL]);
    for (int d = 0; d < D; d++) {
        float v;
        if (bands && nbands >= 2 * D) v = 0.5f * (gl_c01(bands[2 * d]) + gl_c01(bands[2 * d + 1]));
        else v = level * (0.6f + 0.4f * gl_sin(M.t * (0.7f + 0.23f * (float)d) + (float)d));
        M.acc[d] += v;
    }
    M.acc_n++;
    /* a token every sixteenth note (the analyser's tempo), else every 1/8 s; Trigger: one now */
    float bpm = gl_clampf(P[P_BPM], 0.0f, 400.0f);
    float period = bpm >= 50.0f ? gl_clampf(15.0f / bpm, 0.06f, 0.3f) : 0.125f;
    M.t_tok += dt;
    M.pass_t += dt;
    const int trig = P[P_TRIG] > 0.5f, kick = P[P_KICK] > 0.5f;
    M.wave_dt = fminf(0.055f, period / 7.0f);                    /* the wave reaches OUT before the next token */
    if (M.t_tok >= period) {                                     /* tokens from the tempo alone (v2) */
        M.t_tok = fmodf(M.t_tok, period);
        token();
    }
    {                                                            /* the panels ease toward the newest values */
        const float ek = fminf(1.0f, dt * 12.0f);
        /* (before the first token the RMS are 0: show nothing yet, never 0 x inf = NaN, which the easing would keep) */
        const float iq = M.sq > 1e-9f ? 1.0f / M.sq : 0.0f, i1 = M.s1 > 1e-9f ? 1.0f / M.s1 : 0.0f;
        const float i2 = M.s2 > 1e-9f ? 1.0f / M.s2 : 0.0f;
        for (int i = 0; i < T; i++)
            for (int d = 0; d < D; d++) M.dX[i][d] += (M.X[i][d] - M.dX[i][d]) * ek;
        for (int i = 0; i < 3 * D; i++)
            for (int j = 0; j < D; j++) M.dcq[i][j] += (M.cq[i][j] * iq - M.dcq[i][j]) * ek;
        for (int h = 0; h < NH; h++)
            for (int i = 0; i < T; i++)
                for (int j = 0; j < T; j++) M.dA[h][i][j] += (M.A[h][i][j] - M.dA[h][i][j]) * ek;
        float hm = 1e-6f;
        for (int i = 0; i < HID; i++) hm = fmaxf(hm, fabsf(M.h1[i]));
        for (int i = 0; i < HID; i++) {
            M.dact[i] += (fabsf(M.h1[i]) / hm - M.dact[i]) * ek;
            for (int j = 0; j < D; j++) M.dc1[i][j] += (M.c1[i][j] * i1 - M.dc1[i][j]) * ek;
        }
        for (int i = 0; i < D; i++) {
            M.dout[i] += (M.out[i] - M.dout[i]) * ek;
            for (int j = 0; j < HID; j++) M.dc2[i][j] += (M.c2[i][j] * i2 - M.dc2[i][j]) * ek;
        }
    }
    if (kick) M.kick_glow = 1.0f;
    if (trig) M.trig_glow = 1.0f;
    M.kick_glow = fmaxf(0.0f, M.kick_glow - dt * 3.0f);
    M.trig_glow = fmaxf(0.0f, M.trig_glow - dt * 1.2f);
    M.glow_out += (M.surprise - M.glow_out) * fminf(1.0f, dt * 6.0f);

    /* colours: the foreground for positive contributions, its complement for negative */
    g_pos[0] = gl_c01(P[P_FG_R]); g_pos[1] = gl_c01(P[P_FG_G]); g_pos[2] = gl_c01(P[P_FG_B]);
    if (g_pos[0] + g_pos[1] + g_pos[2] < 0.15f) g_pos[0] = g_pos[1] = g_pos[2] = 0.7f;
    {
        const float hk = 0.57735026919f, hc = cosf(3.1415927f), hs = sinf(3.1415927f);
        float dotp = hk * (g_pos[0] + g_pos[1] + g_pos[2]);
        float kx[3] = { hk * (g_pos[2] - g_pos[1]), hk * (g_pos[0] - g_pos[2]), hk * (g_pos[1] - g_pos[0]) };
        for (int c = 0; c < 3; c++) g_neg[c] = gl_c01(g_pos[c] * hc + kx[c] * hs + hk * dotp * (1.0f - hc));
        float gp = g_pos[0] + g_pos[1] + g_pos[2], gn = g_neg[0] + g_neg[1] + g_neg[2];
        if (gn < 0.3f * gp) for (int c = 0; c < 3; c++) g_neg[c] = 0.45f * (1.0f - g_pos[c]) + 0.2f;  /* greys */
    }

    /* the camera: the whole wall from the front; knob 1 brings it close, where it glides along the flow from panel
     * to panel; knob 2 orbits (centre still) */
    float k1 = gl_c01(P[P_K1]), k2 = (gl_c01(P[P_K2]) - 0.5f) * 2.0f;
    k2 = fabsf(k2) < 0.04f ? 0.0f : copysignf((fabsf(k2) - 0.04f) / 0.96f, k2);
    M.yaw += (k2 * 1.3f - M.yaw) * fminf(1.0f, dt * 2.0f);   /* a wall: turned up to 75 degrees, never from behind */
    const float dist = 36.0f * powf(0.3f, k1);                   /* 36 (the wall) .. 11 (a panel) */
    M.cam_u += dt * 0.035f;
    float u = gl_fract(M.cam_u) * (float)NPANEL, f = gl_fract(u);
    int pa = (int)u % NPANEL, pb = (pa + 1) % NPANEL;
    f = f * f * (3.0f - 2.0f * f);
    float tx = PANELS[pa].cx + (PANELS[pb].cx - PANELS[pa].cx) * f, ty = PANELS[pa].cy + (PANELS[pb].cy - PANELS[pa].cy) * f;
    float close = gl_c01((36.0f - dist) / 22.0f);
    float sway = 0.25f * gl_sin(M.t * 0.07f);
    gl_camera(&g_cam, M.yaw + sway, 0.12f + 0.05f * gl_sin(M.t * 0.05f), dist, 0.9f, tx * close, -1.0f + (ty + 1.0f) * close,
              0.0f);

    const float lift = 1.0f + 0.5f * M.trig_glow;
    /* EMBED: the tokens (x = time, newest on the right; y = the spectrum, low at the bottom) */
    {
        float w = pass_wave(0) * lift;
        for (int i = 0; i < T; i++)
            for (int d = 0; d < D; d++) {
                float v = gl_clampf(M.dX[i][d] * 0.8f, -1.0f, 1.0f);
                float nw = (i == T - 1) ? 0.6f + 0.8f * w : 0.35f;
                dot(0, ((float)i + 0.5f) / T, ((float)d + 0.5f) / D, v * nw, 0.06f, 0.30f);
            }
        panel_frame(0, w);
    }
    /* Q K V: 48 outputs (x) by 16 inputs (y) */
    {
        float w = pass_wave(1) * lift;
        for (int i = 0; i < 3 * D; i++)
            for (int j = 0; j < D; j++)
                dot(1, ((float)i + 0.5f) / (3 * D), ((float)j + 0.5f) / D, M.dcq[i][j] * (0.35f + 0.65f * w), 0.05f, 0.19f);
        panel_frame(1, w);
    }
    /* ATTENTION: the four heads' maps, 2 x 2 (x = the token attended to, y = the token attending, newest at the top),
     * the newest row lit by the wave */
    {
        float w = pass_wave(2) * lift;
        for (int h = 0; h < NH; h++) {
            float ox = (h & 1) ? 0.5f : 0.0f, oy = (h & 2) ? 0.0f : 0.5f;
            for (int i = 0; i < T; i++) {
                float mx = 1e-6f;                            /* each row by its strongest: where this token looked */
                for (int j = 0; j <= i; j++) mx = fmaxf(mx, M.dA[h][i][j]);
                for (int j = 0; j <= i; j++) {
                    float a = M.dA[h][i][j] / mx;
                    float glow = (i == T - 1) ? 0.6f + 0.8f * w : 0.6f;
                    dot(2, ox + ((float)j + 0.5f) / T * 0.46f + 0.02f, oy + ((float)i + 0.5f) / T * 0.46f + 0.02f,
                        a * a * glow, 0.05f, 0.24f);
                }
            }
        }
        panel_frame(2, w);
    }
    /* MLP UP: 48 hidden (x) by 16 inputs (y) */
    {
        float w = pass_wave(3) * lift;
        for (int i = 0; i < HID; i++)                            /* a column = a hidden unit, lit by its activity */
            for (int j = 0; j < D; j++)
                dot(3, ((float)i + 0.5f) / HID, ((float)j + 0.5f) / D,
                    1.4f * M.dc1[i][j] * (0.35f + 0.65f * w) * (0.4f + 0.9f * M.dact[i]), 0.05f, 0.19f);
        panel_frame(3, w);
    }
    /* MLP DOWN: 48 hidden (x) by 16 outputs (y) */
    {
        float w = pass_wave(4) * lift;
        for (int i = 0; i < D; i++)
            for (int j = 0; j < HID; j++)
                dot(4, ((float)j + 0.5f) / HID, ((float)i + 0.5f) / D, 1.4f * M.dc2[i][j] * (0.35f + 0.65f * w), 0.05f,
                    0.19f);
        panel_frame(4, w);
    }
    /* OUT: the guess at the next token as 16 bars, and the surprise lighting the panel */
    {
        float w = pass_wave(5) * lift;
        const Panel *Pp = &PANELS[5];
        for (int d = 0; d < D; d++) {
            float v = gl_clampf(M.dout[d] * 0.5f, -1.0f, 1.0f);
            float x = Pp->cx + (((float)d + 0.5f) / D - 0.5f) * Pp->w, y0 = Pp->cy;
            const float *col = v >= 0.0f ? g_pos : g_neg;
            float b = 0.55f + 1.1f * w;
            float cc[3] = { col[0] * b, col[1] * b, col[2] * b };
            gl_line3(&g_cam, x, y0, Pp->cz, x, y0 + v * Pp->h * 0.45f, Pp->cz, 0.16f, cc, cc);
        }
        float s = M.glow_out * (1.0f + M.kick_glow);
        gl_sprite3(&g_cam, Pp->cx, Pp->cy, Pp->cz - 0.6f, Pp->w * 0.28f, g_neg[0] * s * 0.35f, g_neg[1] * s * 0.35f, g_neg[2] * s * 0.35f);
        panel_frame(5, w);
    }
    beam(0, 1, 0, 2);
    beam(1, 2, 0, 2);
    beam(2, 3, 1, 1);
    beam(3, 4, 2, 0);
    beam(4, 5, 2, 0);
    {                                                         /* the residual stream: EMBED straight down to OUT */
        const Panel *A = &PANELS[0], *B2 = &PANELS[5];
        float dim[3] = { g_pos[0] * 0.06f + 0.02f, g_pos[1] * 0.06f + 0.02f, g_pos[2] * 0.06f + 0.02f };
        gl_line3(&g_cam, A->cx, A->cy - A->h * 0.5f - 0.5f, A->cz, B2->cx, B2->cy + B2->h * 0.5f + 0.5f, B2->cz, 0.04f, dim, dim);
    }
    /* knob 3: bloom up to the middle, a dead band, then trails from 0.6 */
    const float bloom_amt = k3 < 0.5f ? k3 * 2.0f * 1.5f : 1.5f;
    const float decay = k3 >= 0.6f ? 0.35f + (k3 - 0.6f) * (0.55f / 0.4f) : 0.0f;
    const float bg[3] = { gl_c01(P[P_BG_R]), gl_c01(P[P_BG_G]), gl_c01(P[P_BG_B]) };
    gl_end(bloom_amt, decay, 1.0f, bg, P[P_PREFILLED] > 0.5f);
    return 0;
}

/* for tests and DEBUG: glow's stats, then tokens so far, the surprise, the newest token's first head's strongest
 * attention */
void fx_stats(float *out, int n) {
    float v[11];
    gl_stats(v);
    v[8] = (float)M.ntok;
    v[9] = M.surprise;
    float mx = 0.0f;
    for (int j = 0; j < T; j++) mx = fmaxf(mx, M.A[0][T - 1][j]);
    v[10] = mx;
    for (int i = 0; i < n && i < 11; i++) out[i] = v[i];
}
