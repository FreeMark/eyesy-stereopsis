/* fluid.c - the kernel of the EYESY mode "S - Ink": glowing ink in water, stirred by the music. A real fluid simulation
 * (Jos Stam's "stable fluids") on the EYESY's CPU, in the Swarm Visualizer's light (glow.h's tone curve, bloom and
 * second core).
 *
 * The water: a velocity field on a 128 x 72 grid. Each frame: the stirring (jets, kicks) adds to it; vorticity
 * confinement puts back the small curls the grid would smooth away (knob 1: more of it = more turbulent); it carries
 * itself along (semi-Lagrangian advection); a pressure solve makes it incompressible (so it swirls instead of
 * piling up). The ink: red, green and blue dye on a grid twice as fine, carried by the water and fading (knob 3: how
 * long it lingers).
 * The music: six jets around the middle, each playing one part of the spectrum (bass to treble) - the louder its
 * part, the harder it squirts ink of its colour (the foreground colour, the others spread around it on the colour
 * wheel). The jets turn around the middle on knob 2. Every kick drops a splash of ink with a burst; Trigger pours a
 * big one in the middle, every colour.
 * Drawn: the ink, bilinearly upsampled, plus a soft bloom of its brightest parts, through the tone curve over the
 * background. */
#include "glow.h"

#define FL_VERSION 1
#define VX 128                     /* velocity grid */
#define VY 72
#define DX 192                     /* dye grid (1.5x: finer than the water, and plenty for a 320 x 180 picture) */
#define DY 108
#define BX 48                      /* bloom grid */
#define BY 27
#define NJET 6
#define JACOBI 14

enum {
    P_DT, P_K1, P_K2, P_K3, P_K4, P_K5,
    P_FG_R, P_FG_G, P_FG_B, P_BG_R, P_BG_G, P_BG_B,
    P_LEVEL, P_BASS, P_MID, P_TREBLE, P_BEAT, P_KICK, P_TRIG, P_PREFILLED,
    P_WAVE_HZ,
    P_COUNT
};

#define VI(x, y) ((y) * (VX + 2) + (x))  /* velocity cells with a border of one */
#define NV ((VX + 2) * (VY + 2))

static float g_u[NV], g_v[NV], g_u0[NV], g_v0[NV], g_pa[NV], g_pb[NV], g_div[NV], g_curl[NV];
static float *g_p = g_pa, *g_p2 = g_pb;          /* the pressure solve ping-pongs between the two */
static float g_dyeA[DX * DY * 3], g_dyeB[DX * DY * 3];
static float *g_dye = g_dyeA, *g_dye0 = g_dyeB;  /* the ink now / the ink it is carried from (swapped each frame) */
static float g_bl[BX * BY * 3], g_bl2[BX * BY * 3];
static uint16_t g_lu[DX * DY * 3];  /* light to draw, x 256, at the dye grid */

static struct {
    int ready;
    float t, rot, kick_glow;
    float jet_lvl[NJET], jet_mean[NJET];
    int splash_rr;
    double prof[4], prof_n;        /* DEBUG: ms per stage (stirring + curl/confine/advect, pressure, ink, draw) */
} F;

int fx_version(void) { return FL_VERSION; }
int fx_param_count(void) { return P_COUNT; }
int fx_ready(void) { return F.ready; }
int fx_threads(int n) { return gl_set_threads(n); }
void fx_trails_reset(void) { gl_trails_reset(); }
float fx_bench(int which, int n, float r) { return gl_bench(which, n, r); }

int fx_init(int n, int seed) {
    (void)n;
    memset(&F, 0, sizeof F);
    memset(g_u, 0, sizeof g_u);
    memset(g_v, 0, sizeof g_v);
    memset(g_pa, 0, sizeof g_pa);
    memset(g_pb, 0, sizeof g_pb);
    memset(g_dyeA, 0, sizeof g_dyeA);
    memset(g_dyeB, 0, sizeof g_dyeB);
    gl_rng_state = 0x68E31DA4u ^ ((uint32_t)seed * 2654435761u);
    if (!gl_rng_state) gl_rng_state = 1u;
    F.ready = 1;
    return 0;
}

/* ---------------------------------------------------------------------------------------- the water */
/* walls: no flow through the edges (the border cells mirror the inside, the normal component negated) */
static void walls(float *u, float *v) {
    for (int y = 1; y <= VY; y++) {
        u[VI(0, y)] = -u[VI(1, y)]; v[VI(0, y)] = v[VI(1, y)];
        u[VI(VX + 1, y)] = -u[VI(VX, y)]; v[VI(VX + 1, y)] = v[VI(VX, y)];
    }
    for (int x = 1; x <= VX; x++) {
        u[VI(x, 0)] = u[VI(x, 1)]; v[VI(x, 0)] = -v[VI(x, 1)];
        u[VI(x, VY + 1)] = u[VI(x, VY)]; v[VI(x, VY + 1)] = -v[VI(x, VY)];
    }
    u[VI(0, 0)] = u[VI(1, 1)]; v[VI(0, 0)] = v[VI(1, 1)];
    u[VI(VX + 1, 0)] = u[VI(VX, 1)]; v[VI(VX + 1, 0)] = v[VI(VX, 1)];
    u[VI(0, VY + 1)] = u[VI(1, VY)]; v[VI(0, VY + 1)] = v[VI(1, VY)];
    u[VI(VX + 1, VY + 1)] = u[VI(VX, VY)]; v[VI(VX + 1, VY + 1)] = v[VI(VX, VY)];
}

static void scalar_edges(float *p) {
    for (int y = 1; y <= VY; y++) { p[VI(0, y)] = p[VI(1, y)]; p[VI(VX + 1, y)] = p[VI(VX, y)]; }
    for (int x = 0; x <= VX + 1; x++) { p[VI(x, 0)] = p[VI(x, 1)]; p[VI(x, VY + 1)] = p[VI(x, VY)]; }
}

/* a field sampled at (x, y) in cells (1..VX, 1..VY are the inside), bilinear, clamped to the border */
static inline float samp(const float *f, float x, float y) {
    x = gl_clampf(x, 0.5f, (float)VX + 0.5f);
    y = gl_clampf(y, 0.5f, (float)VY + 0.5f);
    int i = (int)x, j = (int)y;
    float fx = x - (float)i, fy = y - (float)j;
    const float *r0 = f + VI(i, j), *r1 = f + VI(i, j + 1);
    return (r0[0] + (r0[1] - r0[0]) * fx) * (1.0f - fy) + (r1[0] + (r1[1] - r1[0]) * fx) * fy;
}

typedef struct { float dt, eps, fade, clear; } Step;
static Step g_step;

static void do_curl(void *ctx, int y0, int y1) {
    (void)ctx;
    for (int y = y0 + 1; y < y1 + 1; y++)
        for (int x = 1; x <= VX; x++)
            g_curl[VI(x, y)] = 0.5f * ((g_v[VI(x + 1, y)] - g_v[VI(x - 1, y)]) - (g_u[VI(x, y + 1)] - g_u[VI(x, y - 1)]));
}

/* vorticity confinement: a push along N x curl, N the unit gradient of |curl| - the curls the grid loses come back */
static void do_confine(void *ctx, int y0, int y1) {
    (void)ctx;
    const float k = g_step.dt * g_step.eps;
    for (int y = y0 + 1; y < y1 + 1; y++) {
        if (y < 2 || y > VY - 1) continue;
        for (int x = 2; x <= VX - 1; x++) {
            float gx = fabsf(g_curl[VI(x + 1, y)]) - fabsf(g_curl[VI(x - 1, y)]);
            float gy = fabsf(g_curl[VI(x, y + 1)]) - fabsf(g_curl[VI(x, y - 1)]);
            float len = sqrtf(gx * gx + gy * gy) + 1e-5f, w = g_curl[VI(x, y)];
            g_u[VI(x, y)] += k * (gy / len) * w;
            g_v[VI(x, y)] -= k * (gx / len) * w;
        }
    }
}

static void do_advect_vel(void *ctx, int y0, int y1) {
    (void)ctx;
    const float dt = g_step.dt;
    for (int y = y0 + 1; y < y1 + 1; y++)
        for (int x = 1; x <= VX; x++) {
            float px = (float)x - dt * g_u0[VI(x, y)], py = (float)y - dt * g_v0[VI(x, y)];
            g_u[VI(x, y)] = samp(g_u0, px, py) * 0.9995f;
            g_v[VI(x, y)] = samp(g_v0, px, py) * 0.9995f;
        }
}

static void do_div(void *ctx, int y0, int y1) {
    (void)ctx;
    for (int y = y0 + 1; y < y1 + 1; y++)
        for (int x = 1; x <= VX; x++)
            g_div[VI(x, y)] = -0.5f * (g_u[VI(x + 1, y)] - g_u[VI(x - 1, y)] + g_v[VI(x, y + 1)] - g_v[VI(x, y - 1)]);
}

static void do_jacobi(void *ctx, int y0, int y1) {
    (void)ctx;
    for (int y = y0 + 1; y < y1 + 1; y++)
        for (int x = 1; x <= VX; x++)
            g_p2[VI(x, y)] = 0.25f * (g_div[VI(x, y)] + g_p[VI(x - 1, y)] + g_p[VI(x + 1, y)] + g_p[VI(x, y - 1)] +
                                      g_p[VI(x, y + 1)]);
}

static void do_gradient(void *ctx, int y0, int y1) {
    (void)ctx;
    for (int y = y0 + 1; y < y1 + 1; y++)
        for (int x = 1; x <= VX; x++) {
            g_u[VI(x, y)] -= 0.5f * (g_p[VI(x + 1, y)] - g_p[VI(x - 1, y)]);
            g_v[VI(x, y)] -= 0.5f * (g_p[VI(x, y + 1)] - g_p[VI(x, y - 1)]);
        }
}

/* the ink: each dye cell looks back along the water's flow (the velocity there, bilinear from the water's grid) */
static void do_advect_dye(void *ctx, int y0, int y1) {
    (void)ctx;
    const float fade = g_step.fade, kx = (float)VX / (float)DX, ky = (float)VY / (float)DY;
    const float su = g_step.dt / kx, sv = g_step.dt / ky;       /* water cells a frame -> dye cells */
    for (int y = y0; y < y1; y++) {
        const float vy = ((float)y + 0.5f) * ky + 0.5f;
        for (int x = 0; x < DX; x++) {
            const float vx = ((float)x + 0.5f) * kx + 0.5f;
            float sx = (float)x - samp(g_u, vx, vy) * su, sy = (float)y - samp(g_v, vx, vy) * sv;   /* back */
            sx = gl_clampf(sx, 0.0f, (float)DX - 1.001f);
            sy = gl_clampf(sy, 0.0f, (float)DY - 1.001f);
            int i = (int)sx, j = (int)sy;
            float fx = sx - (float)i, fy = sy - (float)j;
            const float *a = g_dye0 + 3 * (j * DX + i), *b = a + 3 * DX;
            float *o = g_dye + 3 * (y * DX + x);
            /* near the walls the ink drains away (an open tank): it would pool along them otherwise */
            int ex = x < DX - 1 - x ? x : DX - 1 - x, ey = y < DY - 1 - y ? y : DY - 1 - y, e = ex < ey ? ex : ey;
            float edge = e >= 9 ? 1.0f : 0.90f + 0.10f * (float)e / 9.0f;
            for (int c = 0; c < 3; c++) {
                float top = a[c] + (a[3 + c] - a[c]) * fx, bot = b[c] + (b[3 + c] - b[c]) * fx;
                /* the half-life, and a small loss besides: faint ink clears, so the tank does not fill with haze */
                o[c] = gl_clampf((top + (bot - top) * fy) * fade * edge - g_step.clear, 0.0f, 64.0f);   /* NaN -> 0 */
            }
        }
    }
}

/* a splat: velocity (vx, vy) and ink (col) around (x, y) (0..1 across, 0..1 down), radius r (fraction of the height) */
static void splat(float x, float y, float vx, float vy, const float col[3], float r) {
    float cx = x * (float)VX + 0.5f, cy = y * (float)VY + 0.5f, rr = r * (float)VY;
    int R = (int)(rr * 2.5f) + 1;
    for (int j = (int)cy - R; j <= (int)cy + R; j++) {
        if (j < 1 || j > VY) continue;
        for (int i = (int)cx - R; i <= (int)cx + R; i++) {
            if (i < 1 || i > VX) continue;
            float dx = (float)i - cx, dy = (float)j - cy, w = expf(-(dx * dx + dy * dy) / (rr * rr));
            g_u[VI(i, j)] += vx * w;
            g_v[VI(i, j)] += vy * w;
        }
    }
    float dcx = x * (float)DX, dcy = y * (float)DY, dr = r * (float)DY;
    int DR = (int)(dr * 2.5f) + 1;
    for (int j = (int)dcy - DR; j <= (int)dcy + DR; j++) {
        if (j < 0 || j >= DY) continue;
        for (int i = (int)dcx - DR; i <= (int)dcx + DR; i++) {
            if (i < 0 || i >= DX) continue;
            float dx = (float)i + 0.5f - dcx, dy = (float)j + 0.5f - dcy, w = expf(-(dx * dx + dy * dy) / (dr * dr));
            float *o = g_dye + 3 * (j * DX + i);
            for (int c = 0; c < 3; c++) o[c] = fminf(o[c] + col[c] * w, 12.0f);
        }
    }
}

static void hue_turn(const float in[3], float turns, float out[3]) {
    const float hk = 0.57735026919f;
    float a = turns * 6.2831853f, hc = cosf(a), hs = sinf(a);
    float dot = hk * (in[0] + in[1] + in[2]);
    float kx[3] = { hk * (in[2] - in[1]), hk * (in[0] - in[2]), hk * (in[1] - in[0]) };
    for (int c = 0; c < 3; c++) out[c] = gl_c01(in[c] * hc + kx[c] * hs + hk * dot * (1.0f - hc));
}

/* ---------------------------------------------------------------------------------------- drawing */
typedef struct {
    int W, H;
    int x0[GL_ROW_MAX];
    uint8_t xw[GL_ROW_MAX];
} Draw;
static Draw g_draw;

/* the bloom's source: the ink's bright part, 4 x 4 dye cells to one */
static void do_bloom_src(void *ctx, int j0, int j1) {
    (void)ctx;
    for (int j = j0; j < j1; j++)
        for (int i = 0; i < BX; i++)
            for (int c = 0; c < 3; c++) {
                float s = 0.0f;
                for (int y = 0; y < 4; y++)
                    for (int x = 0; x < 4; x++) s += g_dye[3 * ((j * 4 + y) * DX + i * 4 + x) + c];
                g_bl[3 * (j * BX + i) + c] = fmaxf(s * (1.0f / 16.0f) - 0.4f, 0.0f);
            }
}

/* the light to draw at the dye grid: the ink + the bloom (upsampled from its coarse grid), x 256 */
static void do_light(void *ctx, int y0, int y1) {
    const float *bamt = (const float *)ctx;
    for (int y = y0; y < y1; y++) {
        float by = ((float)y + 0.5f) * ((float)BY / (float)DY) - 0.5f;
        by = gl_clampf(by, 0.0f, (float)BY - 1.001f);
        int j = (int)by;
        float fy = by - (float)j;
        for (int x = 0; x < DX; x++) {
            float bx = ((float)x + 0.5f) * ((float)BX / (float)DX) - 0.5f;
            bx = gl_clampf(bx, 0.0f, (float)BX - 1.001f);
            int i = (int)bx;
            float fx = bx - (float)i;
            const float *a = g_bl + 3 * (j * BX + i), *b = a + 3 * BX;
            const float *d = g_dye + 3 * (y * DX + x);
            uint16_t *o = g_lu + 3 * (y * DX + x);
            for (int c = 0; c < 3; c++) {
                float bl = (a[c] + (a[3 + c] - a[c]) * fx) * (1.0f - fy) + (b[c] + (b[3 + c] - b[c]) * fx) * fy;
                float v = d[c] + bl * bamt[0];
                o[c] = (uint16_t)fminf(fmaxf(v, 0.0f) * GL_LFX, 65535.0f);
            }
        }
    }
}

/* output rows [y0, y1): the light bilinearly from the dye grid, through the tone tables */
static void do_rows(void *ctx, int y0, int y1) {
    (void)ctx;
    const Draw *D = &g_draw;
    uint16_t row[DX * 3];
    uint32_t tmp[GL_ROW_MAX];                                 /* a line before the block upscale (render scale > 1) */
    const uint8_t *tr = gl_tm8[0], *tg = gl_tm8[1], *tb = gl_tm8[2];
    for (int yy = y0; yy < y1; yy++) {
        float sy = ((float)yy + 0.5f) * ((float)DY / (float)D->H) - 0.5f;
        sy = gl_clampf(sy, 0.0f, (float)DY - 1.001f);
        int j = (int)sy;
        uint32_t wy = (uint32_t)((sy - (float)j) * 256.0f), vy = 256 - wy;
        const uint16_t *a = g_lu + 3 * j * DX, *b = a + 3 * DX;
        for (int k = 0; k < 3 * DX; k++) row[k] = (uint16_t)((a[k] * vy + b[k] * wy) >> 8);
        uint32_t *orow = GF.down == 1 ? (uint32_t *)(GF.dst + (size_t)yy * (size_t)GF.dpitch) : NULL;
        uint32_t *out = orow ? orow : tmp;
        for (int x = 0; x < D->W; x++) {
            const uint16_t *p = row + 3 * D->x0[x];
            uint32_t w = D->xw[x], v = 256 - w;
            uint32_t r = (p[0] * v + p[3] * w) >> 8, g = (p[1] * v + p[4] * w) >> 8, bb = (p[2] * v + p[5] * w) >> 8;
            r = r < GL_TM8_N - 1 ? r : GL_TM8_N - 1;
            g = g < GL_TM8_N - 1 ? g : GL_TM8_N - 1;
            bb = bb < GL_TM8_N - 1 ? bb : GL_TM8_N - 1;
            out[x] = ((uint32_t)tr[r] << GF.sh_r) | ((uint32_t)tg[g] << GF.sh_g) | ((uint32_t)tb[bb] << GF.sh_b);
        }
        if (!orow) {                                          /* render scale > 1: block upscale */
            for (int r = 0; r < GF.down; r++) {
                int oy = yy * GF.down + r;
                if (oy >= GF.dH) break;
                uint32_t *o = (uint32_t *)(GF.dst + (size_t)oy * (size_t)GF.dpitch);
                for (int x = 0; x < D->W; x++)
                    for (int k = 0; k < GF.down; k++) {
                        int ox = x * GF.down + k;
                        if (ox < GF.dW) o[ox] = out[x];
                    }
            }
        }
    }
}

int fx_frame(uint8_t *dst, int dW, int dH, int dpitch, int down, int sh_r, int sh_g, int sh_b, float *acc, int W, int H,
             float *bloom, int BW, int BH, const float *fft, int nfft, float fft_hz, const float *wave, int nwave,
             const float *bands, int nbands, const float *P) {
    (void)fft_hz;
    if (!F.ready) return -9;
    if (!P) return -1;
    if (nfft < 0 || nfft > 65536 || nwave < 0 || nwave > 65536 || nbands < 0 || nbands > 256 || (nfft > 0 && !fft) ||
        (nwave > 0 && !wave) || (nbands > 0 && !bands))
        return -3;
    /* gl_begin checks the frame and the buffers (the primitives stay unused: this mode draws a field) */
    int rc = gl_begin(dst, dW, dH, dpitch, down, sh_r, sh_g, sh_b, acc, W, H, bloom, BW, BH, 0);
    if (rc) return rc;
    GF.ok = 0;
    double t0 = gl_now_ms();
    const float dt = gl_clampf(P[P_DT], 0.0f, 0.05f);
    F.t += dt;
    if (F.t > 1.0e5f) F.t = 0.0f;
    const float level = gl_c01(P[P_LEVEL]), bass = gl_c01(P[P_BASS]);
    const float k1 = gl_c01(P[P_K1]), k3 = gl_c01(P[P_K3]);
    float k2 = (gl_c01(P[P_K2]) - 0.5f) * 2.0f;
    k2 = fabsf(k2) < 0.08f ? 0.0f : copysignf((fabsf(k2) - 0.08f) / 0.92f, k2);
    F.rot += k2 * 0.5f * dt;
    if (F.rot > 1.0e4f || F.rot < -1.0e4f) F.rot = 0.0f;
    float fg[3] = { gl_c01(P[P_FG_R]), gl_c01(P[P_FG_G]), gl_c01(P[P_FG_B]) };
    if (fg[0] + fg[1] + fg[2] < 0.15f) { fg[0] = 0.2f; fg[1] = 0.6f; fg[2] = 1.0f; }
    /* ---- the stirring: six jets around the middle, each a sixth of the spectrum (bass first) */
    const float aspect = (float)VX / (float)VY;
    for (int k = 0; k < NJET; k++) {
        float e;
        if (bands && nbands >= 30) {
            e = 0.0f;
            for (int b = k * 5; b < k * 5 + 5; b++) e += gl_c01(bands[b]);
            e *= 0.2f;
        } else {
            e = level * (0.6f + 0.4f * gl_sin(F.t * (1.1f + 0.37f * (float)k) + (float)k));
        }
        F.jet_mean[k] += (e - F.jet_mean[k]) * fminf(1.0f, dt * 0.8f);
        /* a steady pour with its part's loudness, and more when it rises above its own running level */
        float lift = 0.35f * e + fmaxf(0.0f, e - F.jet_mean[k] * 0.7f);
        F.jet_lvl[k] += (lift - F.jet_lvl[k]) * fminf(1.0f, dt * 12.0f);
        float a = F.rot + (float)k * (6.2831853f / NJET);
        float r = 0.30f;
        float x = 0.5f + cosf(a) * r / aspect, y = 0.5f + sinf(a) * r;
        /* squirting inward and around: a vortex pair across the middle */
        float dxv = -cosf(a) * 0.55f - sinf(a) * 0.8f, dyv = -sinf(a) * 0.55f + cosf(a) * 0.8f;
        float s = F.jet_lvl[k] * (70.0f + 90.0f * k1);
        if (s > 0.3f) {
            float col[3];
            hue_turn(fg, ((float)k - 2.5f) * 0.11f, col);
            float ink = F.jet_lvl[k] * 1.1f * dt * 60.0f * 0.2f;
            float c[3] = { col[0] * ink, col[1] * ink, col[2] * ink };
            splat(x, y, dxv * s * dt * 60.0f * 0.05f, dyv * s * dt * 60.0f * 0.05f, c, 0.035f + 0.02f * F.jet_lvl[k]);
        }
    }
    if (P[P_KICK] > 0.5f) {                                           /* a splash where the next jet stands */
        int k = F.splash_rr++ % NJET;
        float a = F.rot + (float)k * (6.2831853f / NJET) + 0.5f;
        float x = 0.5f + cosf(a) * 0.22f / aspect, y = 0.5f + sinf(a) * 0.22f;
        float col[3];
        hue_turn(fg, ((float)k - 2.5f) * 0.11f, col);
        float amp = 0.9f + 1.4f * bass;
        float c[3] = { col[0] * amp, col[1] * amp, col[2] * amp };
        splat(x, y, cosf(a) * 5.0f * amp, sinf(a) * 5.0f * amp, c, 0.06f);
        F.kick_glow = 1.0f;
    }
    if (P[P_TRIG] > 0.5f)                                             /* Trigger: every colour, in the middle */
        for (int k = 0; k < NJET; k++) {
            float a = (float)k * (6.2831853f / NJET) + F.t;
            float col[3];
            hue_turn(fg, (float)k / NJET, col);
            float c[3] = { col[0] * 2.5f, col[1] * 2.5f, col[2] * 2.5f };
            splat(0.5f + cosf(a) * 0.05f, 0.5f + sinf(a) * 0.08f, cosf(a) * 8.0f, sinf(a) * 8.0f, c, 0.05f);
        }
    F.kick_glow = fmaxf(0.0f, F.kick_glow - dt * 3.0f);
    /* ---- the water */
    g_step.dt = dt * 60.0f;                                   /* cells per frame at 60 fps: velocities in cells/frame */
    g_step.eps = 0.08f + 0.9f * k1 * k1;                      /* knob 1: calm -> curly */
    g_step.fade = powf(0.5f, dt / (0.3f * powf(50.0f, k3)));  /* knob 3: the ink's half-life, 0.3 s .. 15 s */
    g_step.clear = 0.0028f * (1.0f - 0.8f * k3) * dt * 60.0f;
    double pa = gl_now_ms();
    walls(g_u, g_v);
    gl_parallel_for(do_curl, NULL, VY, 12);
    gl_parallel_for(do_confine, NULL, VY, 12);
    walls(g_u, g_v);
    memcpy(g_u0, g_u, sizeof g_u);
    memcpy(g_v0, g_v, sizeof g_v);
    gl_parallel_for(do_advect_vel, NULL, VY, 12);
    walls(g_u, g_v);
    gl_parallel_for(do_div, NULL, VY, 12);
    scalar_edges(g_div);
    double pb = gl_now_ms();
    for (int it = 0; it < JACOBI; it++) {                     /* Jacobi from last frame's pressure, both cores */
        gl_parallel_for(do_jacobi, NULL, VY, 12);
        scalar_edges(g_p2);
        float *t = g_p; g_p = g_p2; g_p2 = t;
    }
    gl_parallel_for(do_gradient, NULL, VY, 12);
    double pc = gl_now_ms();
    walls(g_u, g_v);
    for (int i = 0; i < NV; i++) {                            /* keep the numbers sane whatever comes in */
        if (!(fabsf(g_u[i]) < 400.0f)) g_u[i] = 0.0f;
        if (!(fabsf(g_v[i]) < 400.0f)) g_v[i] = 0.0f;
        if (!(fabsf(g_p[i]) < 1.0e5f)) g_p[i] = 0.0f;
    }
    { float *t = g_dye0; g_dye0 = g_dye; g_dye = t; }        /* carry from the ink as it is into the other buffer */
    gl_parallel_for(do_advect_dye, NULL, DY, 8);
    double t1 = gl_now_ms();
    F.prof[0] += pb - pa;
    F.prof[1] += pc - pb;
    F.prof[2] += t1 - pc;
    F.prof[3] += pa - t0;
    F.prof_n += 1.0;
    /* ---- the bloom: the ink's bright part on a coarse grid (4 x 4 cells), blurred twice (box, radius 2) */
    float bamt = 0.9f + 0.4f * F.kick_glow;
    gl_parallel_for(do_bloom_src, NULL, BY, 4);
    for (int pass = 0; pass < 2; pass++) {
        for (int j = 0; j < BY; j++)
            for (int i = 0; i < BX; i++)
                for (int c = 0; c < 3; c++) {
                    float s = 0.0f;
                    for (int k = -2; k <= 2; k++) {
                        int x = i + k < 0 ? 0 : (i + k >= BX ? BX - 1 : i + k);
                        s += g_bl[3 * (j * BX + x) + c];
                    }
                    g_bl2[3 * (j * BX + i) + c] = s * 0.2f;
                }
        for (int j = 0; j < BY; j++)
            for (int i = 0; i < BX; i++)
                for (int c = 0; c < 3; c++) {
                    float s = 0.0f;
                    for (int k = -2; k <= 2; k++) {
                        int y = j + k < 0 ? 0 : (j + k >= BY ? BY - 1 : j + k);
                        s += g_bl2[3 * (y * BX + i) + c];
                    }
                    g_bl[3 * (j * BX + i) + c] = s * 0.2f;
                }
    }
    gl_parallel_for(do_light, &bamt, DY, 8);
    /* ---- to the screen */
    const float bg[3] = { gl_c01(P[P_BG_R]), gl_c01(P[P_BG_G]), gl_c01(P[P_BG_B]) };
    gl_tone_update(1.0f, bg);
    Draw *D = &g_draw;
    D->W = GF.W;
    D->H = GF.H;
    for (int x = 0; x < D->W; x++) {
        float sx = ((float)x + 0.5f) * ((float)DX / (float)D->W) - 0.5f;
        sx = gl_clampf(sx, 0.0f, (float)DX - 1.001f);
        D->x0[x] = (int)sx;
        D->xw[x] = (uint8_t)((sx - (float)(int)sx) * 255.0f);
    }
    gl_parallel_for(do_rows, NULL, GF.H, 16);
    double t2 = gl_now_ms();
    gl_stage[0] += t1 - t0;                                   /* (reported as bin: the simulation) */
    gl_stage[1] += 0.0;
    gl_stage[2] += t2 - t1;                                   /* (final: bloom + drawing) */
    gl_stage[3] += 1.0;
    return 0;
}

/* for tests and DEBUG: glow's timing slots (the simulation as "bin", drawing as "final"), then ms per frame of the
 * stirring + velocity steps, the pressure solve, the ink */
void fx_stats(float *out, int n) {
    float v[11];
    gl_stats(v);
    double ink = 0.0;
    for (int i = 0; i < DX * DY * 3; i++) ink += g_dye[i];
    float vmax = 0.0f;
    for (int i = 0; i < NV; i++) vmax = fmaxf(vmax, fabsf(g_u[i]) + fabsf(g_v[i]));
    (void)ink;
    (void)vmax;
    double pn = F.prof_n > 0.0 ? F.prof_n : 1.0;
    v[8] = (float)((F.prof[0] + F.prof[3]) / pn);
    v[9] = (float)(F.prof[1] / pn);
    v[10] = (float)(F.prof[2] / pn);
    memset(F.prof, 0, sizeof F.prof);
    F.prof_n = 0.0;
    for (int i = 0; i < n && i < 11; i++) out[i] = v[i];
}
