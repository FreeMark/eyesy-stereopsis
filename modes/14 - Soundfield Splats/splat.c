/* splat.c - native Gaussian-splat renderer for the EYESY mode "S - Soundfield Splats".
 *
 * The mode compiles this file ON the EYESY (gcc 12, armhf, Cortex-A53) the first time it runs, caches
 * the .so in its own folder, and calls it through ctypes. It writes straight into the screen surface's
 * pixel memory, so a bad write would corrupt the video engine: every argument is validated, every write
 * is bounds-checked against the caller's pitch/height, and float values only become indices after
 * explicit range checks (the build deliberately avoids -ffast-math, which would delete NaN guards).
 *
 * Physics (slowed ~1000x): each source emits its signal history outward at a "speed of sound".
 * A particle at distance r sees that source's signal from r*spu history steps ago, softened by
 * r0/(r+r0) (spherical spreading), and is displaced along the radial direction by it (air parcels move
 * along the direction of travel). Pressure > 0 = compression, < 0 = rarefaction.
 *
 * Rendering: emission-only (additive, order-independent) 2D Gaussian splats with depth of field (the
 * circle of confusion grows the splat, energy is conserved) into a low-res float accumulator.
 * v5: particles are projected first, counting-sorted into 32x32 screen tiles, then splatted tile by
 * tile, so each tile's accumulator block (~12 KB) stays in L1 instead of every splat missing to DRAM
 * (the accumulator is megabytes; the CM3's L2 is 512 KB). One fused pass then tone-maps (1 - e^-x) over
 * the background, decays the accumulator for trails, and block-upscales straight into the destination.
 * Measured on the EYESY: its engine alone spends ~31 ms/frame at 1280x720, so every ms here is fps.
 */
#include <math.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#define MAXR 24            /* max splat radius, low-res pixels */
#define TM_N 1024          /* tone-map LUT */
#define TM_MAX 10.0f
#define G_N 1024           /* Gaussian LUT: exp(-x), x in [0, G_MAX) */
#define G_MAX 9.0f
#define ROW_MAX 8192
#define TILE 32
#define TILES_MAX 4096

typedef struct { float sx, sy, k2, ar, ag, ab; int R, tile; } Splat;

static float g_tm[TM_N];
static float g_gauss[G_N];
static int g_ready = 0;
static uint32_t g_row[ROW_MAX];
static Splat *g_splats = NULL;
static int *g_order = NULL;
static int g_cap = 0;
static int g_tstart[TILES_MAX + 1];

static void luts_init(void) {
    for (int i = 0; i < TM_N; i++) g_tm[i] = 1.0f - expf(-(TM_MAX * (float)i) / (float)(TM_N - 1));
    for (int i = 0; i < G_N; i++) g_gauss[i] = expf(-(G_MAX * (float)i) / (float)(G_N - 1));
    g_ready = 1;
}

static inline float clampf(float v, float lo, float hi) {
    if (!(v == v)) return lo;                 /* NaN -> lo */
    return v < lo ? lo : (v > hi ? hi : v);
}

/* exp(-q) for q >= 0 via LUT; 0 beyond G_MAX or for NaN */
static inline float gauss(float q) {
    const float s = (float)(G_N - 1) / G_MAX;
    float f = q * s;
    if (!(f >= 0.0f && f < (float)(G_N - 1))) return 0.0f;
    return g_gauss[(int)f];
}

int sf_version(void) { return 5; }

static inline float hist_at(const float *h, int hlen, float k) {
    if (!(k >= 0.0f)) k = 0.0f;
    if (!(k < (float)(hlen - 1))) return 0.0f;
    int i = (int)k;
    float f = k - (float)i;
    return h[i] * (1.0f - f) + h[i + 1] * f;
}

enum {
    P_YAW, P_PITCH, P_CAMDIST, P_FOVPX, P_FOCUS, P_APERTURE,
    P_DISPGAIN, P_BASEB, P_WAVEB, P_SPLATPX, P_DECAY, P_EXPOSURE,
    P_STEPS_PER_UNIT, P_R0, P_WAVE_SAT, P_SRC_BRIGHT,
    P_COLPOS_R, P_COLPOS_G, P_COLPOS_B,
    P_COLNEG_R, P_COLNEG_G, P_COLNEG_B,
    P_COLBASE_R, P_COLBASE_G, P_COLBASE_B,
    P_BG_R, P_BG_G, P_BG_B,
    P_COUNT
};

int sf_param_count(void) { return P_COUNT; }

static int valid_shift(int s) { return s == 0 || s == 8 || s == 16 || s == 24; }

/* dst: 32-bit pixels, dW x dH, dpitch bytes per row. acc: W*H*3 floats with W = ceil(dW/down),
 * H = ceil(dH/down). Returns 0, or <0 if arguments were rejected (then nothing is written). */
int sf_render(uint8_t *dst, int dW, int dH, int dpitch, int down, int sh_r, int sh_g, int sh_b,
              float *acc, int W, int H,
              int n, const float *pos, const float *seed,
              int nsrc, const float *src, const float *src_level,
              const float *hist, int hlen, const float *P)
{
    if (!dst || !acc || !P || !hist || (n > 0 && (!pos || !seed)) || (nsrc > 0 && (!src || !src_level))) return -1;
    if (dW <= 0 || dH <= 0 || dW > ROW_MAX || dH > ROW_MAX || dpitch < dW * 4) return -2;
    if (n < 0 || n > 200000 || nsrc < 0 || nsrc > 8 || hlen < 2 || hlen > 100000) return -3;
    if (!valid_shift(sh_r) || !valid_shift(sh_g) || !valid_shift(sh_b) || sh_r == sh_g || sh_r == sh_b || sh_g == sh_b) return -4;
    if ((dpitch & 3) || ((uintptr_t)dst & 3)) return -5;
    if (down < 1 || down > 8 || W != (dW + down - 1) / down || H != (dH + down - 1) / down) return -6;
    const int TX = (W + TILE - 1) / TILE, TY = (H + TILE - 1) / TILE, NT = TX * TY;
    if (NT > TILES_MAX) return -8;
    const int total = n + nsrc;
    if (total > g_cap) {                              /* grow the splat buffers once; never shrink */
        Splat *nb = (Splat *)realloc(g_splats, sizeof(Splat) * (size_t)total);
        if (!nb) return -7;
        g_splats = nb;
        int *no = (int *)realloc(g_order, sizeof(int) * (size_t)total);
        if (!no) return -7;
        g_order = no;
        g_cap = total;
    }
    if (!g_ready) luts_init();

    /* camera + look */
    const float cyaw = cosf(P[P_YAW]), syaw = sinf(P[P_YAW]);
    const float cpit = cosf(P[P_PITCH]), spit = sinf(P[P_PITCH]);
    const float camd = P[P_CAMDIST], fov = P[P_FOVPX], focus = P[P_FOCUS], aper = P[P_APERTURE];
    const float halfW = 0.5f * (float)W, halfH = 0.5f * (float)H;
    const float spu = P[P_STEPS_PER_UNIT], r0 = P[P_R0] > 1e-4f ? P[P_R0] : 1e-4f;
    const float dispg = P[P_DISPGAIN], baseb = P[P_BASEB], waveb = P[P_WAVEB], splat = P[P_SPLATPX];
    const float sat = P[P_WAVE_SAT] > 1e-6f ? P[P_WAVE_SAT] : 1.0f;
    float cpos[3], cneg[3], cbase[3];                  /* clamped copies: a NaN colour must never reach acc */
    for (int c = 0; c < 3; c++) {
        cpos[c] = clampf(P[P_COLPOS_R + c], 0.0f, 1.0f);
        cneg[c] = clampf(P[P_COLNEG_R + c], 0.0f, 1.0f);
        cbase[c] = clampf(P[P_COLBASE_R + c], 0.0f, 1.0f);
    }

    /* 1a. physics + projection -> compact splat list, counted per tile */
    memset(g_tstart, 0, sizeof(int) * (size_t)(NT + 1));
    int ns = 0;
    for (int i = 0; i < total; i++) {
        float x, y, z, bright, sscale, cr, cg, cb;
        if (i < n) {
            const float *p = pos + 3 * (size_t)i;
            float dx = 0.0f, dy = 0.0f, dz = 0.0f, pres = 0.0f;
            for (int s = 0; s < nsrc; s++) {
                float ex = p[0] - src[3 * s], ey = p[1] - src[3 * s + 1], ez = p[2] - src[3 * s + 2];
                float r = sqrtf(ex * ex + ey * ey + ez * ez) + 1e-6f;
                float v = hist_at(hist + (size_t)s * (size_t)hlen, hlen, r * spu) * (r0 / (r + r0));
                pres += v;
                float k = v / r;
                dx += ex * k; dy += ey * k; dz += ez * k;
            }
            x = p[0] + dx * dispg; y = p[1] + dy * dispg; z = p[2] + dz * dispg;
            float t = clampf(fabsf(pres) / sat, 0.0f, 1.0f);
            const float *cw = pres >= 0.0f ? cpos : cneg;
            cr = cbase[0] + (cw[0] - cbase[0]) * t;
            cg = cbase[1] + (cw[1] - cbase[1]) * t;
            cb = cbase[2] + (cw[2] - cbase[2]) * t;
            /* crests glow with a squared curve, troughs stay dim -> wavefronts read as rings */
            bright = baseb + waveb * t * t * (pres >= 0.0f ? 1.0f : 0.4f);   /* dark near every zero crossing */
            /* dust thinning: barely-lit particles cost as much as bright ones; draw one in three at 3x
               brightness - same average glow, a third of the cost in quiet passages */
            if (t < 0.08f) {
                if (i % 3) continue;
                bright *= 3.0f;
            }
            sscale = 0.7f + 0.6f * seed[i];
        } else {
            int s = i - n;
            x = src[3 * s]; y = src[3 * s + 1]; z = src[3 * s + 2];
            float lv = clampf(src_level[s], 0.0f, 1.0f);
            cr = cpos[0]; cg = cpos[1]; cb = cpos[2];
            bright = P[P_SRC_BRIGHT] * (0.35f + lv);
            sscale = 2.2f + 2.5f * lv;
        }
        float x1 = cyaw * x + syaw * z;
        float z1 = -syaw * x + cyaw * z;
        float y2 = cpit * y - spit * z1;
        float zc = spit * y + cpit * z1 + camd;
        if (!(zc > 0.05f)) continue;
        float sxp = halfW + fov * x1 / zc;
        float syp = halfH - fov * y2 / zc;
        if (!(fabsf(sxp) < 1.0e6f && fabsf(syp) < 1.0e6f)) continue;
        float sigma = splat * sscale * (camd / zc);
        float coc = aper * fabsf(zc - focus) / zc;
        float sig2 = sigma * sigma + coc * coc;
        if (!(sig2 < 1.0e6f)) continue;
        if (sig2 < 0.16f) sig2 = 0.16f;
        float amp = bright * (sigma * sigma + 0.1f) / (sig2 + 0.1f);
        if (!(amp > 1e-5f && amp < 1.0e6f)) continue;
        int R = (int)ceilf(3.0f * sqrtf(sig2));
        if (R > MAXR) R = MAXR;
        if (R < 1) R = 1;
        int cx = (int)floorf(sxp), cyy = (int)floorf(syp);
        if (cx + R < 0 || cyy + R < 0 || cx - R >= W || cyy - R >= H) continue;
        int tx = cx < 0 ? 0 : (cx >= W ? TX - 1 : cx / TILE);
        int ty = cyy < 0 ? 0 : (cyy >= H ? TY - 1 : cyy / TILE);
        Splat *sp = &g_splats[ns++];
        sp->sx = sxp; sp->sy = syp; sp->k2 = 0.5f / sig2;
        sp->ar = amp * cr; sp->ag = amp * cg; sp->ab = amp * cb;
        sp->R = R; sp->tile = ty * TX + tx;
        g_tstart[sp->tile + 1]++;
    }

    /* 1b. counting sort by tile */
    for (int t = 0; t < NT; t++) g_tstart[t + 1] += g_tstart[t];
    for (int i = 0; i < ns; i++) g_order[g_tstart[g_splats[i].tile]++] = i;   /* g_tstart[t] now = end of tile t */

    /* 1c. splat in tile order */
    float wx[2 * MAXR + 1];
    for (int oi = 0; oi < ns; oi++) {
        const Splat *sp = &g_splats[g_order[oi]];
        const float sxp = sp->sx, syp = sp->sy, k2 = sp->k2;
        const int R = sp->R;
        int cx = (int)floorf(sxp), cyy = (int)floorf(syp);
        int xa = cx - R, xb = cx + R, ya = cyy - R, yb = cyy + R;
        if (xa < 0) xa = 0;
        if (ya < 0) ya = 0;
        if (xb >= W) xb = W - 1;
        if (yb >= H) yb = H - 1;
        if (xa > xb || ya > yb) continue;
        const int wn = xb - xa + 1;                       /* <= 2R+1 <= 2*MAXR+1 */
        for (int j = 0; j < wn; j++) { float d = (float)(xa + j) + 0.5f - sxp; wx[j] = gauss(d * d * k2); }
        const float ar = sp->ar, ag = sp->ag, ab = sp->ab;
        for (int yy = ya; yy <= yb; yy++) {
            float d = (float)yy + 0.5f - syp;
            float wy = gauss(d * d * k2);
            if (wy <= 0.0f) continue;
            float *row = acc + 3 * ((size_t)yy * (size_t)W + (size_t)xa);
            for (int j = 0; j < wn; j++) {
                float w = wx[j] * wy;
                row[3 * j] += ar * w; row[3 * j + 1] += ag * w; row[3 * j + 2] += ab * w;
            }
        }
    }

    /* 2. per-call packed LUTs: tone curve over the background, pre-shifted into pixel position */
    static uint32_t lr[TM_N], lg[TM_N], lb[TM_N];
    const float bgr = clampf(P[P_BG_R], 0.0f, 1.0f), bgg = clampf(P[P_BG_G], 0.0f, 1.0f), bgb = clampf(P[P_BG_B], 0.0f, 1.0f);
    for (int i = 0; i < TM_N; i++) {
        lr[i] = (uint32_t)(uint8_t)(255.0f * (bgr + (1.0f - bgr) * g_tm[i]) + 0.5f) << sh_r;
        lg[i] = (uint32_t)(uint8_t)(255.0f * (bgg + (1.0f - bgg) * g_tm[i]) + 0.5f) << sh_g;
        lb[i] = (uint32_t)(uint8_t)(255.0f * (bgb + (1.0f - bgb) * g_tm[i]) + 0.5f) << sh_b;
    }

    /* 3. fused: tone map + trail decay + block upscale into the destination */
    const float expo = clampf(P[P_EXPOSURE], 0.0f, 1.0e3f) * (float)(TM_N - 1) / TM_MAX;
    const float decay = clampf(P[P_DECAY], 0.0f, 0.995f);
    const float tmax = (float)(TM_N - 1);
    for (int yy = 0; yy < H; yy++) {
        float *a = acc + 3 * (size_t)yy * (size_t)W;
        for (int xx = 0; xx < W; xx++) {
            float qr = a[3 * xx] * expo, qg = a[3 * xx + 1] * expo, qb = a[3 * xx + 2] * expo;
            int ir = qr > 0.0f ? (qr < tmax ? (int)qr : TM_N - 1) : 0;
            int ig = qg > 0.0f ? (qg < tmax ? (int)qg : TM_N - 1) : 0;
            int ib = qb > 0.0f ? (qb < tmax ? (int)qb : TM_N - 1) : 0;
            g_row[xx] = lr[ir] | lg[ig] | lb[ib];
            a[3 * xx] *= decay; a[3 * xx + 1] *= decay; a[3 * xx + 2] *= decay;
        }
        int oy0 = yy * down;
        if (oy0 >= dH) break;
        uint32_t *o = (uint32_t *)(dst + (size_t)oy0 * (size_t)dpitch);
        int ox = 0;
        for (int xx = 0; xx < W && ox < dW; xx++) {
            uint32_t v = g_row[xx];
            for (int k = 0; k < down && ox < dW; k++) o[ox++] = v;
        }
        for (int r = 1; r < down && oy0 + r < dH; r++)
            memcpy(dst + (size_t)(oy0 + r) * (size_t)dpitch, o, (size_t)dW * 4u);
    }
    return 0;
}
