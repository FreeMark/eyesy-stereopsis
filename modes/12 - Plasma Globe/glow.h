/* glow.h - the renderer shared by the newer EYESY modes (S - Soundfield Splats 2, S - Circuit, S - Transformer,
 * S - Phase Space, S - Plasma Globe): glowing points and lines on the EYESY's CPU, in the look of S - Swarm Visualizer.
 * Each mode's kernel (<mode>.c, next to a copy of this file) includes it, builds its scene each frame from sprites
 * and lines, and hands the frame over; the mode compiles the pair ON the EYESY and calls it through ctypes.
 * The canonical copy is modes/_lib/glow.h in the development tree (tools/sync_glow.py copies it into the modes).
 *
 * It writes straight into the screen surface's pixel memory, so every argument is validated, every write is
 * bounds-checked against the caller's pitch/height, and float values only become indices after explicit range
 * checks (no -ffast-math: it would delete the NaN guards).
 *
 * A frame:
 *   gl_begin(...)            the destination, the buffers, the render size; < 0 = rejected, draw nothing
 *   gl_sprite / gl_line ...  soft round LEDs (free.vet's drone sprite: hot core + halo, as the Swarm) and soft
 *                            lines with that same profile across them and a colour gradient along them
 *   gl_end(...)              bloom, the tone curve over the background, trails, block upscale into the screen
 * Everything light is additive, so order does not matter. Light is in the tone curve's units: 1 at a sprite's
 * centre = 1 - e^-1 of the way from the background to white (at exposure 1).
 *
 * Inside (the Swarm Visualizer's pipeline, see swarm.c and the JOURNAL): primitives are binned into 32 x 32 tiles
 * (a copy in each tile a primitive's box touches; lines are cut into short pieces first); the bloom's source is
 * deposited straight from the primitives at quarter resolution, blurred in integers; then band by band (a row of
 * tiles) each lit tile's light gathers in a 6 KB buffer that stays in L1, is tone mapped with byte tables and written
 * to the screen. Trails keep a persistent 16-bit accumulator instead. The bands go to two threads (the engine's and
 * a worker), each band computed exactly as on one, so the picture does not depend on the thread count.
 */
#ifndef GLOW_H
#define GLOW_H

#include <math.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <pthread.h>

#define GL_TILE 32
#define GL_TILES_MAX 4096
#define GL_ROW_MAX 8192
#define GL_MAXR 24                 /* the largest sprite radius / line half-width, render px */
#define GL_SPR2_N 1024             /* the profile, by squared distance */
#define GL_TM8_N 4096              /* tone tables: light 0 .. 16 in steps of 1/256 */
#define GL_LFX 256.0f              /* light in the accumulators is fixed point: 1.0 = 256 */
#define GL_PIECE 24.0f             /* lines are cut into pieces at most this long (px): each touches few tiles */
#define GL_PRIMS_MAX 200000

typedef struct {
    float x0, y0;                  /* sprite: centre; line: start */
    float ux, uy;                  /* line: unit direction */
    float len, inv_r;              /* line: length (px); 1 / radius (sprite) or 1 / half-width (line) */
    uint16_t c0[3];                /* light at the profile's peak, x 256: the sprite / the line at its start */
    uint16_t c1[3];                /* line: at its end */
    int16_t xa, xb, ya, yb;        /* its box on the render, clipped */
    uint8_t kind;                  /* 0 sprite, 1 line of one colour, 2 line with a gradient */
} GPrim;

typedef struct {                   /* a camera: world -> render px (the Swarm Visualizer's convention) */
    float cy, sy, cp, sp;          /* yaw, pitch */
    float dist, focal, halfW, halfH;
    float tx, ty, tz;              /* the point it looks at */
} GCam;

/* ---------------------------------------------------------------------------------------- state */
static float gl_spr2[GL_SPR2_N];
static uint16_t gl_prof16[GL_SPR2_N];      /* the profile, its peak = 65535 */
static float gl_prof_max;                  /* the profile's peak */
static float gl_spr_energy;                /* light of a sprite of peak 1 and radius 1, over its disc */
static float gl_line_energy;               /* light of a line of peak 1 and half-width 1, per px of length */
static int gl_luts_ready = 0;
static float gl_t8[GL_TM8_N];
static float gl_t8_E = -1.0f;
static uint8_t gl_tm8[3][GL_TM8_N];
static float gl_tm8_bg[3] = { -1.0f, -1.0f, -1.0f };

static GPrim *gl_list = NULL;              /* this frame's primitives */
static int gl_n = 0, gl_cap = 0;
static GPrim *gl_bin = NULL;               /* ... binned: a copy in every tile its box touches */
static int gl_bin_cap = 0;
static int gl_tstart[GL_TILES_MAX + 1];
static int gl_tfill[GL_TILES_MAX];
static unsigned char gl_tflag[GL_TILES_MAX];   /* 1 = primitives reach it, 2 = bloom reaches it */
static unsigned char gl_tres[GL_TILES_MAX];    /* trails: light is left in the tile's part of the accumulator */
static const float *gl_acc_last = NULL;
static int gl_acc_w = 0, gl_acc_h = 0, gl_acc_dirty = 1;
static int gl_overflow = 0;                /* primitives dropped this frame (the list is full) */

static struct {
    uint8_t *dst;
    int dW, dH, dpitch, down, sh_r, sh_g, sh_b;
    int W, H, TX, TY, NT, BW, BH;
    float *acc, *bloom;
    int bloom_on;                  /* the bloom source is collected this frame */
    int ok;                        /* gl_begin accepted the frame */
} GF;

static double gl_stage[4];         /* ms since the last gl_stats: bin, bloom, final pass; frames */
static double gl_tiles[3];         /* tiles per frame: lit, bloom only, dark */

static double gl_now_ms(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return ts.tv_sec * 1000.0 + ts.tv_nsec / 1.0e6;
}

static inline float gl_clampf(float v, float lo, float hi) {
    if (!(v == v)) return lo;                                 /* NaN -> lo */
    return v < lo ? lo : (v > hi ? hi : v);
}

static inline float gl_c01(float v) { return fminf(fmaxf(v, 0.0f), 1.0f); }   /* NaN -> 0 */

static void gl_luts_init(void) {
    /* free.vet's drone.frag, by the squared distance q = u^2 (u = r / radius): core = smoothstep(0.5, 0.0, r/2),
     * halo = core^2.4, light = core x (0.35 + 1.15 halo) */
    double e = 0.0;
    for (int i = 0; i < GL_SPR2_N; i++) {
        float u = sqrtf(((float)i + 0.5f) / (float)GL_SPR2_N), t = 1.0f - u;
        float core = t * t * (3.0f - 2.0f * t);
        gl_spr2[i] = core * (0.35f + powf(core, 2.4f) * 1.15f);
        e += gl_spr2[i];                                      /* uniform in u^2 = uniform in area */
    }
    gl_prof_max = 0.0f;
    for (int i = 0; i < GL_SPR2_N; i++) gl_prof_max = gl_spr2[i] > gl_prof_max ? gl_spr2[i] : gl_prof_max;
    for (int i = 0; i < GL_SPR2_N; i++) gl_prof16[i] = (uint16_t)(gl_spr2[i] / gl_prof_max * 65535.0f + 0.5f);
    gl_spr_energy = (float)(3.14159265358979 * e / GL_SPR2_N) / gl_prof_max;
    double le = 0.0;                                          /* across a line: the integral of the profile over */
    for (int i = 0; i < 2048; i++) {                          /* u in (-1, 1), the peak = 1 */
        float u = ((float)i + 0.5f) / 1024.0f - 1.0f;
        int qi = (int)(u * u * GL_SPR2_N);
        le += qi < GL_SPR2_N ? gl_spr2[qi] / gl_prof_max : 0.0f;
    }
    gl_line_energy = (float)(le / 1024.0);
    gl_luts_ready = 1;
}

/* ---------------------------------------------------------------------------------------- the tone curve */
/* light -> pixel: channel = 255 (bg + (1 - bg)(1 - e^-(E light))) + 0.5, in byte tables over the fixed-point light.
 * Rebuilt when E or the background changes (rarely) */
static void gl_tone_update(float E, const float bg[3]) {
    if (E != gl_t8_E) {
        for (int i = 0; i < GL_TM8_N; i++) gl_t8[i] = 1.0f - expf(-E * (float)i / GL_LFX);
        gl_t8_E = E;
        gl_tm8_bg[0] = -1.0f;
    }
    if (bg[0] != gl_tm8_bg[0] || bg[1] != gl_tm8_bg[1] || bg[2] != gl_tm8_bg[2]) {
        for (int c = 0; c < 3; c++) {
            for (int i = 0; i < GL_TM8_N; i++)
                gl_tm8[c][i] = (uint8_t)(255.0f * (bg[c] + (1.0f - bg[c]) * gl_t8[i]) + 0.5f);
            gl_tm8_bg[c] = bg[c];
        }
    }
}

/* n pixels: light from a (if use_a) + bloom already expanded per pixel in bl (if use_b), through the tone tables,
 * packed at the given shifts (inlined with constants for the EYESY's XRGB 16/8/0) */
static inline __attribute__((always_inline)) void gl_tone_run(uint32_t *restrict out, const uint16_t *restrict a,
                                                              const uint16_t *restrict bl, int n, int use_a, int use_b,
                                                              int sr, int sg, int sb) {
    const uint8_t *tr = gl_tm8[0], *tg = gl_tm8[1], *tb = gl_tm8[2];
    for (int i = 0; i < n; i++) {
        uint32_t r = 0, g = 0, b = 0;
        if (use_a) {
            r = a[3 * i];
            g = a[3 * i + 1];
            b = a[3 * i + 2];
        }
        if (use_b) {
            r += bl[3 * i];
            g += bl[3 * i + 1];
            b += bl[3 * i + 2];
        }
        r = r < GL_TM8_N - 1 ? r : GL_TM8_N - 1;
        g = g < GL_TM8_N - 1 ? g : GL_TM8_N - 1;
        b = b < GL_TM8_N - 1 ? b : GL_TM8_N - 1;
        out[i] = ((uint32_t)tr[r] << sr) | ((uint32_t)tg[g] << sg) | ((uint32_t)tb[b] << sb);
    }
}

/* groups of 4 pixels with light AND bloom on XRGB, fused: pixels 4j .. 4j+3 read bloom cells j-1, j, j+1 once and
 * weigh them 5, 7, 1, 3 eighths (what the general path's bx0 / bw8 give them, so the result is the same) */
static void gl_tone_ab4(uint32_t *out, const uint16_t *a, const uint16_t *bc, int j0, int groups) {
    const uint8_t *tr = gl_tm8[0], *tg = gl_tm8[1], *tb = gl_tm8[2];
    const uint16_t *c = bc + 3 * (j0 - 1);
    for (int gi = 0; gi < groups; gi++, a += 12, out += 4, c += 3) {
        const uint32_t L0 = c[0], L1 = c[1], L2 = c[2], M0 = c[3], M1 = c[4], M2 = c[5], R0 = c[6], R1 = c[7], R2 = c[8];
#define GL_PIX(i, br, bgr, bbl)                                                                          \
        do {                                                                                             \
            uint32_t r_ = a[3 * (i)] + (br), g_ = a[3 * (i) + 1] + (bgr), b_ = a[3 * (i) + 2] + (bbl);   \
            r_ = r_ < GL_TM8_N - 1 ? r_ : GL_TM8_N - 1;                                                  \
            g_ = g_ < GL_TM8_N - 1 ? g_ : GL_TM8_N - 1;                                                  \
            b_ = b_ < GL_TM8_N - 1 ? b_ : GL_TM8_N - 1;                                                  \
            out[i] = ((uint32_t)tr[r_] << 16) | ((uint32_t)tg[g_] << 8) | (uint32_t)tb[b_];             \
        } while (0)
        GL_PIX(0, (3 * L0 + 5 * M0) >> 3, (3 * L1 + 5 * M1) >> 3, (3 * L2 + 5 * M2) >> 3);
        GL_PIX(1, (L0 + 7 * M0) >> 3, (L1 + 7 * M1) >> 3, (L2 + 7 * M2) >> 3);
        GL_PIX(2, (7 * M0 + R0) >> 3, (7 * M1 + R1) >> 3, (7 * M2 + R2) >> 3);
        GL_PIX(3, (5 * M0 + 3 * R0) >> 3, (5 * M1 + 3 * R1) >> 3, (5 * M2 + 3 * R2) >> 3);
#undef GL_PIX
    }
}

/* one run [x0, x1) of a row (x1 - x0 <= TILE): f = 1 the light (its first pixel at column ax), 2 the bloom (cells bc,
 * 4x bilinear by bx0 / bw8), 3 both */
static void gl_tone_gen(uint32_t *orow, const uint16_t *a, int ax, const uint16_t *bc, const int *bx0,
                        const uint8_t *bw8, int x0, int x1, int f, int sh_r, int sh_g, int sh_b) {
    uint16_t bl[3 * GL_TILE];
    const int n = x1 - x0;
    if (n <= 0 || n > GL_TILE)
        return;
    if (f & 2)
        for (int i = 0; i < n; i++) {
            const uint16_t *c = bc + 3 * bx0[x0 + i];
            uint32_t w = bw8[x0 + i], v = 8 - w;
            bl[3 * i] = (uint16_t)((c[0] * v + c[3] * w) >> 3);
            bl[3 * i + 1] = (uint16_t)((c[1] * v + c[4] * w) >> 3);
            bl[3 * i + 2] = (uint16_t)((c[2] * v + c[5] * w) >> 3);
        }
    uint32_t *out = orow + x0;
    const uint16_t *ap = (f & 1) ? a + 3 * (x0 - ax) : NULL;
    if (sh_r == 16 && sh_g == 8 && sh_b == 0) {
        if (f == 1) gl_tone_run(out, ap, NULL, n, 1, 0, 16, 8, 0);
        else if (f == 2) gl_tone_run(out, NULL, bl, n, 0, 1, 16, 8, 0);
        else gl_tone_run(out, ap, bl, n, 1, 1, 16, 8, 0);
    } else {
        gl_tone_run(out, ap, bl, n, f & 1, (f & 2) >> 1, sh_r, sh_g, sh_b);
    }
}

static void gl_tone_fx(uint32_t *orow, const uint16_t *a, int ax, const uint16_t *bc, const int *bx0,
                       const uint8_t *bw8, int BW, int x0, int x1, int f, int sh_r, int sh_g, int sh_b) {
    if (f == 3 && sh_r == 16 && sh_g == 8 && sh_b == 0 && (x0 & 3) == 0 && x1 - x0 <= GL_TILE) {
        int ja = x0 >> 2, jb = x1 >> 2;
        ja = ja < 1 ? 1 : ja;
        jb = jb > BW - 1 ? BW - 1 : jb;
        if (jb > ja) {
            gl_tone_gen(orow, a, ax, bc, bx0, bw8, x0, 4 * ja, f, sh_r, sh_g, sh_b);
            gl_tone_ab4(orow + 4 * ja, a + 3 * (4 * ja - ax), bc, ja, jb - ja);
            gl_tone_gen(orow, a, ax, bc, bx0, bw8, 4 * jb, x1, f, sh_r, sh_g, sh_b);
            return;
        }
    }
    gl_tone_gen(orow, a, ax, bc, bx0, bw8, x0, x1, f, sh_r, sh_g, sh_b);
}

/* ---------------------------------------------------------------------------------------- splatting */
static inline uint16_t gl_sat(uint32_t v) { return (uint16_t)(v < 65535u ? v : 65535u); }

/* a sprite into a fixed-point RGB buffer whose pixel (bx, by) is at buf[0], over the clipped rows [ya, yb] and columns
 * [xa, xb]: q = (distance / radius)^2 stepped along a row by integer second differences (x 2^20), so a pixel is
 * integer adds, a table read and three multiplies. Saturating adds: a hot spot clips at 256, it does not wrap */
static inline void gl_splat_sprite(const GPrim *s, int xa, int xb, int ya, int yb, uint16_t *buf, int bx, int by,
                                   int stride) {
    const float QS = 1048576.0f;
    const float is = s->inv_r;                                /* inside the disc |dx|, |dy| < 1: q fits easily */
    const int32_t d2 = (int32_t)(2.0f * is * is * QS);
    const uint32_t fr = s->c0[0], fg = s->c0[1], fb = s->c0[2];
    const float r = 1.0f / is;
    for (int y = ya; y <= yb; y++) {
        float dy = ((float)y + 0.5f - s->y0) * is;
        float dy2 = dy * dy;
        if (dy2 >= 1.0f) continue;
        /* only the row's pixels inside the circle (a sprite's box is 4/pi its disc, more on small ones): pixel centres
         * within hw of the centre; the stepping starts there */
        float hw = r * sqrtf(1.0f - dy2);
        int xs = (int)ceilf(s->x0 - 0.5f - hw), xe = (int)floorf(s->x0 - 0.5f + hw);
        xs = xs < xa ? xa : xs;
        xe = xe > xb ? xb : xe;
        if (xs > xe) continue;
        const float dxs = ((float)xs + 0.5f - s->x0) * is;
        uint16_t *p = buf + (size_t)(y - by) * (size_t)stride + 3 * (size_t)(xs - bx);
        int32_t q = (int32_t)((dxs * dxs + dy2) * QS), d1 = (int32_t)((2.0f * dxs * is + is * is) * QS);
        for (int xx = xs; xx <= xe; xx++, p += 3) {
            int32_t qi = q >> 10;
            q += d1;
            d1 += d2;
            qi &= ~(qi >> 31);                                  /* rounding can dip below 0 at the centre */
            if (qi >= GL_SPR2_N) continue;
            uint32_t w = gl_prof16[qi];
            p[0] = gl_sat(p[0] + ((fr * w) >> 16));
            p[1] = gl_sat(p[1] + ((fg * w) >> 16));
            p[2] = gl_sat(p[2] + ((fb * w) >> 16));
        }
    }
}

/* a line piece over the clipped box: across it the sprite's profile (q = (distance / half-width)^2, quadratic along
 * a row, stepped like the sprite's), along it a ramp over the last half pixel at each end (so pieces laid end to end
 * add up to an even line) and, for kind 2, the colour lerped from c0 to c1. Per row only the span where the line can
 * be lit is visited (from the distance and the ends, solved for x) */
static inline void gl_splat_line(const GPrim *L, int xa, int xb, int ya, int yb, uint16_t *buf, int bx, int by,
                                 int stride) {
    const float ux = L->ux, uy = L->uy, ir = L->inv_r, len = L->len;
    const float QS = 1048576.0f, SF = 65536.0f;
    const float il = len > 1e-3f ? 1.0f / len : 0.0f;
    const float au = uy * ir;                                 /* d(across, in radii) / dx */
    const int grad = L->kind == 2;
    const float c0r = L->c0[0], c0g = L->c0[1], c0b = L->c0[2];
    const float dcr = (float)L->c1[0] - c0r, dcg = (float)L->c1[1] - c0g, dcb = (float)L->c1[2] - c0b;
    for (int y = ya; y <= yb; y++) {
        const float dy = (float)y + 0.5f - L->y0;
        /* across(x) = ((x + 0.5 - x0) uy - dy ux) ir ; along(x) = (x + 0.5 - x0) ux + dy uy  (x = the column) */
        const float k0 = (0.5f - L->x0);
        float lo = (float)xa, hi = (float)xb;
        if (fabsf(uy) > 1e-6f) {                              /* |across| < 1 */
            float xa_ = ((-1.0f / ir) + dy * ux) / uy - k0, xb_ = ((1.0f / ir) + dy * ux) / uy - k0;
            float mn = fminf(xa_, xb_), mx = fmaxf(xa_, xb_);
            lo = fmaxf(lo, mn - 1.0f);
            hi = fminf(hi, mx + 1.0f);
        } else if (fabsf(dy * ux * ir) >= 1.0f) {
            continue;
        }
        if (fabsf(ux) > 1e-6f) {                              /* along in [-0.5, len + 0.5] */
            float xa_ = (-0.5f - dy * uy) / ux - k0, xb_ = (len + 0.5f - dy * uy) / ux - k0;
            float mn = fminf(xa_, xb_), mx = fmaxf(xa_, xb_);
            lo = fmaxf(lo, mn - 1.0f);
            hi = fminf(hi, mx + 1.0f);
        } else {
            float s = dy * uy;
            if (s < -0.5f || s > len + 0.5f) continue;
        }
        if (!(lo <= hi)) continue;
        int x0 = (int)ceilf(lo), x1 = (int)floorf(hi);
        x0 = x0 < xa ? xa : x0;
        x1 = x1 > xb ? xb : x1;
        if (x0 > x1) continue;
        const float px = (float)x0 + 0.5f - L->x0;
        const float d0 = (px * uy - dy * ux) * ir;
        const float s0 = px * ux + dy * uy;
        int32_t q = (int32_t)fminf(d0 * d0 * QS, 2.0e9f);
        int32_t d1 = (int32_t)fminf(fmaxf((2.0f * d0 * au + au * au) * QS, -2.0e9f), 2.0e9f);
        const int32_t dd = (int32_t)(2.0f * au * au * QS);
        int32_t s = (int32_t)(s0 * SF);                       /* along, 16.16 px */
        const int32_t ds = (int32_t)(ux * SF);
        const int32_t lenf = (int32_t)(len * SF);
        uint16_t *p = buf + (size_t)(y - by) * (size_t)stride + 3 * (size_t)(x0 - bx);
        for (int x = x0; x <= x1; x++, p += 3, s += ds) {
            int32_t qi = q >> 10;
            q += d1;
            d1 += dd;
            qi &= ~(qi >> 31);
            if (qi >= GL_SPR2_N) continue;
            /* the end ramps: min(1, s + 0.5, len - s + 0.5), 16.16 */
            int32_t ra = s + 32768, rb = lenf - s + 32768;
            int32_t ramp = ra < rb ? ra : rb;
            if (ramp <= 0) continue;
            uint32_t w = gl_prof16[qi];
            if (ramp < 65536) w = (w * (uint32_t)ramp) >> 16;
            uint32_t fr, fg, fb;
            if (grad) {
                float t = gl_c01((float)s * (1.0f / SF) * il);
                fr = (uint32_t)(c0r + dcr * t);
                fg = (uint32_t)(c0g + dcg * t);
                fb = (uint32_t)(c0b + dcb * t);
            } else {
                fr = L->c0[0];
                fg = L->c0[1];
                fb = L->c0[2];
            }
            p[0] = gl_sat(p[0] + ((fr * w) >> 16));
            p[1] = gl_sat(p[1] + ((fg * w) >> 16));
            p[2] = gl_sat(p[2] + ((fb * w) >> 16));
        }
    }
}

static inline void gl_splat(const GPrim *p, int xa, int xb, int ya, int yb, uint16_t *buf, int bx, int by, int stride) {
    if (p->kind == 0) gl_splat_sprite(p, xa, xb, ya, yb, buf, bx, by, stride);
    else gl_splat_line(p, xa, xb, ya, yb, buf, bx, by, stride);
}

/* line yy of the bloom (quarter-resolution fixed-point U, lerped between its two nearest rows) for cells
 * [c_lo, c_hi]; with the last cell, a copy past the edge (a zero-weight bilinear read touches it) */
static void gl_bloom_row(uint16_t *bc, const uint16_t *U, int BW, int BH, int yy, int c_lo, int c_hi) {
    int y0 = ((yy + 2) >> 2) - 1;
    uint32_t w = (uint32_t)((2 * (yy & 3) + 5) & 7);
    if (y0 < 0) { y0 = 0; w = 0; }
    if (y0 >= BH - 1) { y0 = BH - 1; w = 0; }
    int y1 = y0 + 1 < BH ? y0 + 1 : y0;
    const uint16_t *r0 = U + 3 * (size_t)y0 * (size_t)BW, *r1 = U + 3 * (size_t)y1 * (size_t)BW;
    const uint32_t v = 8 - w;
    for (int k = 3 * c_lo; k < 3 * (c_hi + 1); k++)
        bc[k] = (uint16_t)((r0[k] * v + r1[k] * w) >> 3);
    if (c_hi == BW - 1) {
        bc[3 * BW] = bc[3 * BW - 3];
        bc[3 * BW + 1] = bc[3 * BW - 2];
        bc[3 * BW + 2] = bc[3 * BW - 1];
    }
}

/* ---------------------------------------------------------------------------------------- the frame */
/* dst: 32-bit pixels, dW x dH, dpitch bytes a row. acc: W*H*3 floats with W = ceil(dW/down), H = ceil(dH/down) (the
 * trails keep 16-bit light in its first half). bloom: BW*BH*6 floats, BW = ceil(W/4), BH = ceil(H/4). 0, or < 0 if
 * the arguments are rejected (then gl_end draws nothing) */
static int gl_begin(uint8_t *dst, int dW, int dH, int dpitch, int down, int sh_r, int sh_g, int sh_b, float *acc,
                    int W, int H, float *bloom, int BW, int BH, int bloom_on) {
    GF.ok = 0;
    if (!dst || !acc || !bloom) return -1;
    if (dW <= 0 || dH <= 0 || dW > GL_ROW_MAX || dH > GL_ROW_MAX || dpitch < dW * 4) return -2;
    if (sh_r == sh_g || sh_r == sh_b || sh_g == sh_b) return -4;
    for (int k = 0; k < 3; k++) {
        int s = k == 0 ? sh_r : k == 1 ? sh_g : sh_b;
        if (s != 0 && s != 8 && s != 16 && s != 24) return -4;
    }
    if ((dpitch & 3) || ((uintptr_t)dst & 3)) return -5;
    if (down < 1 || down > 8 || W != (dW + down - 1) / down || H != (dH + down - 1) / down) return -6;
    if (BW != (W + 3) / 4 || BH != (H + 3) / 4) return -6;
    const int TX = (W + GL_TILE - 1) / GL_TILE, TY = (H + GL_TILE - 1) / GL_TILE;
    if (TX * TY > GL_TILES_MAX) return -8;
    if (!gl_luts_ready) gl_luts_init();
    GF.dst = dst; GF.dW = dW; GF.dH = dH; GF.dpitch = dpitch; GF.down = down;
    GF.sh_r = sh_r; GF.sh_g = sh_g; GF.sh_b = sh_b;
    GF.W = W; GF.H = H; GF.TX = TX; GF.TY = TY; GF.NT = TX * TY; GF.BW = BW; GF.BH = BH;
    GF.acc = acc; GF.bloom = bloom;
    GF.bloom_on = bloom_on;
    if (bloom_on) memset(bloom, 0, sizeof(float) * 3 * (size_t)BW * (size_t)BH);
    gl_n = 0;
    gl_overflow = 0;
    GF.ok = 1;
    return 0;
}

static GPrim *gl_new_prim(void) {
    if (gl_n >= gl_cap) {
        if (gl_cap >= GL_PRIMS_MAX) { gl_overflow++; return NULL; }
        int nc = gl_cap ? gl_cap * 2 : 8192;
        nc = nc > GL_PRIMS_MAX ? GL_PRIMS_MAX : nc;
        GPrim *nl = (GPrim *)realloc(gl_list, sizeof(GPrim) * (size_t)nc);
        if (!nl) { gl_overflow++; return NULL; }
        gl_list = nl;
        gl_cap = nc;
    }
    return &gl_list[gl_n++];
}

/* bloom source: light e (already per full-resolution pixel, i.e. / 16 per cell) at (x, y), bilinear over the cells */
static inline void gl_bloom_dep(float x, float y, float er, float eg, float eb) {
    float fx = x * 0.25f - 0.5f, fy = y * 0.25f - 0.5f;
    if (!(fx > -2.0f && fy > -2.0f && fx < (float)GF.BW + 1.0f && fy < (float)GF.BH + 1.0f)) return;
    int gx = (int)floorf(fx), gy = (int)floorf(fy);
    float wx = fx - (float)gx, wy = fy - (float)gy;
    for (int k = 0; k < 4; k++) {
        int cx = gx + (k & 1), cy = gy + (k >> 1);
        if (cx < 0 || cy < 0 || cx >= GF.BW || cy >= GF.BH) continue;
        float w = ((k & 1) ? wx : 1.0f - wx) * ((k >> 1) ? wy : 1.0f - wy);
        float *o = GF.bloom + 3 * ((size_t)cy * (size_t)GF.BW + (size_t)cx);
        o[0] += er * w;
        o[1] += eg * w;
        o[2] += eb * w;
    }
}

static inline uint16_t gl_fx16(float v) { return (uint16_t)fminf(fmaxf(v * GL_LFX, 0.0f), 65535.0f); }

/* a soft round LED at (x, y) (render px), radius r px, light (cr, cg, cb) at its centre */
static void gl_sprite(float x, float y, float r, float cr, float cg, float cb) {
    if (!GF.ok) return;
    if (!(fabsf(x) < 1.0e6f && fabsf(y) < 1.0e6f)) return;           /* NaN, far off */
    r = gl_clampf(r, 0.75f, (float)GL_MAXR);
    cr = gl_clampf(cr, 0.0f, 255.0f); cg = gl_clampf(cg, 0.0f, 255.0f); cb = gl_clampf(cb, 0.0f, 255.0f);
    if (cr + cg + cb <= 0.0f) return;
    int R = (int)ceilf(r);
    int cx = (int)floorf(x), cy = (int)floorf(y);
    if (cx + R < 0 || cy + R < 0 || cx - R >= GF.W || cy - R >= GF.H) return;
    GPrim *p = gl_new_prim();
    if (!p) return;
    p->kind = 0;
    p->x0 = x; p->y0 = y; p->inv_r = 1.0f / r;
    p->c0[0] = gl_fx16(cr); p->c0[1] = gl_fx16(cg); p->c0[2] = gl_fx16(cb);
    p->xa = (int16_t)(cx - R < 0 ? 0 : cx - R);
    p->xb = (int16_t)(cx + R >= GF.W ? GF.W - 1 : cx + R);
    p->ya = (int16_t)(cy - R < 0 ? 0 : cy - R);
    p->yb = (int16_t)(cy + R >= GF.H ? GF.H - 1 : cy + R);
    if (GF.bloom_on) {
        float e = gl_spr_energy * r * r * (1.0f / 16.0f);
        gl_bloom_dep(x, y, cr * e, cg * e, cb * e);
    }
}

/* one piece (<= GL_PIECE px long) */
static void gl_line_piece(float x0, float y0, float x1, float y1, float r, const float c0[3], const float c1[3]) {
    float dx = x1 - x0, dy = y1 - y0, len = sqrtf(dx * dx + dy * dy);
    int R = (int)ceilf(r) + 1;
    float xmin = fminf(x0, x1), xmax = fmaxf(x0, x1), ymin = fminf(y0, y1), ymax = fmaxf(y0, y1);
    int xa = (int)floorf(xmin) - R, xb = (int)floorf(xmax) + R, ya = (int)floorf(ymin) - R, yb = (int)floorf(ymax) + R;
    if (xb < 0 || yb < 0 || xa >= GF.W || ya >= GF.H) return;
    GPrim *p = gl_new_prim();
    if (!p) return;
    int grad = c0[0] != c1[0] || c0[1] != c1[1] || c0[2] != c1[2];
    p->kind = grad ? 2 : 1;
    p->x0 = x0; p->y0 = y0;
    if (len > 1e-4f) { p->ux = dx / len; p->uy = dy / len; }
    else { p->ux = 1.0f; p->uy = 0.0f; len = 0.0f; }
    p->len = len;
    p->inv_r = 1.0f / r;
    for (int k = 0; k < 3; k++) {
        p->c0[k] = gl_fx16(c0[k]);
        p->c1[k] = gl_fx16(c1[k]);
    }
    p->xa = (int16_t)(xa < 0 ? 0 : xa);
    p->xb = (int16_t)(xb >= GF.W ? GF.W - 1 : xb);
    p->ya = (int16_t)(ya < 0 ? 0 : ya);
    p->yb = (int16_t)(yb >= GF.H ? GF.H - 1 : yb);
    if (GF.bloom_on) {                        /* the bloom's source along the piece, every <= 3 px */
        int steps = (int)(len / 3.0f) + 1;
        float seg = len / (float)steps, e = gl_line_energy * r * seg * (1.0f / 16.0f);
        for (int i = 0; i < steps; i++) {
            float t = ((float)i + 0.5f) / (float)steps;
            float cr = c0[0] + (c1[0] - c0[0]) * t, cg = c0[1] + (c1[1] - c0[1]) * t, cb = c0[2] + (c1[2] - c0[2]) * t;
            gl_bloom_dep(x0 + dx * t, y0 + dy * t, cr * e, cg * e, cb * e);
        }
    }
}

/* a soft line from (x0, y0) to (x1, y1) (render px), half-width r px, light c0 at its start and c1 at its end (the
 * same colour for both = one colour). The ends are flat (with a half-pixel ramp), so lines laid end to end make an
 * even polyline */
static void gl_line(float x0, float y0, float x1, float y1, float r, const float c0[3], const float c1[3]) {
    if (!GF.ok) return;
    if (!(fabsf(x0) < 1.0e5f && fabsf(y0) < 1.0e5f && fabsf(x1) < 1.0e5f && fabsf(y1) < 1.0e5f)) return;
    r = gl_clampf(r, 0.6f, (float)GL_MAXR);
    float a[3], b[3];
    for (int k = 0; k < 3; k++) {
        a[k] = gl_clampf(c0[k], 0.0f, 255.0f);
        b[k] = gl_clampf(c1[k], 0.0f, 255.0f);
    }
    if (a[0] + a[1] + a[2] + b[0] + b[1] + b[2] <= 0.0f) return;
    /* clip to the render box (+ the width): long lines that cross it are cut to what shows */
    float m = r + 2.0f, lx0 = -m, ly0 = -m, lx1 = (float)GF.W + m, ly1 = (float)GF.H + m;
    float t0 = 0.0f, t1 = 1.0f, dx = x1 - x0, dy = y1 - y0;
    float pp[4] = { -dx, dx, -dy, dy }, qq[4] = { x0 - lx0, lx1 - x0, y0 - ly0, ly1 - y0 };
    for (int i = 0; i < 4; i++) {                             /* Liang-Barsky */
        if (pp[i] == 0.0f) {
            if (qq[i] < 0.0f) return;
        } else {
            float t = qq[i] / pp[i];
            if (pp[i] < 0.0f) { if (t > t1) return; if (t > t0) t0 = t; }
            else { if (t < t0) return; if (t < t1) t1 = t; }
        }
    }
    float len = sqrtf(dx * dx + dy * dy) * (t1 - t0);
    int n = (int)ceilf(len / GL_PIECE);
    n = n < 1 ? 1 : (n > 512 ? 512 : n);
    for (int i = 0; i < n; i++) {
        float ta = t0 + (t1 - t0) * (float)i / (float)n, tb = t0 + (t1 - t0) * (float)(i + 1) / (float)n;
        float ca[3], cb[3];
        for (int k = 0; k < 3; k++) {
            ca[k] = a[k] + (b[k] - a[k]) * ta;
            cb[k] = a[k] + (b[k] - a[k]) * tb;
        }
        gl_line_piece(x0 + dx * ta, y0 + dy * ta, x0 + dx * tb, y0 + dy * tb, r, ca, cb);
    }
}

/* ---------------------------------------------------------------------------------------- cameras */
/* yaw about the vertical, then pitch (looking down), from dist away from (tx, ty, tz); fov = vertical, radians */
static void gl_camera(GCam *c, float yaw, float pitch, float dist, float fov, float tx, float ty, float tz) {
    c->cy = cosf(yaw); c->sy = sinf(yaw);
    c->cp = cosf(pitch); c->sp = sinf(pitch);
    c->dist = dist;
    c->halfW = 0.5f * (float)GF.W;
    c->halfH = 0.5f * (float)GF.H;
    c->focal = c->halfH / tanf(0.5f * gl_clampf(fov, 0.1f, 3.0f));
    c->tx = tx; c->ty = ty; c->tz = tz;
}

/* world -> render px; returns the depth in front of the camera (<= 0: behind it, and the point is far off-screen) */
static inline float gl_project(const GCam *c, float x, float y, float z, float *sx, float *sy) {
    x -= c->tx; y -= c->ty; z -= c->tz;
    float x1 = c->cy * x - c->sy * z, z1 = c->sy * x + c->cy * z;
    float y2 = c->cp * y - c->sp * z1, z2 = c->sp * y + c->cp * z1;
    float d = c->dist - z2;
    *sx = -1.0e9f;
    *sy = -1.0e9f;
    if (!(d > 1e-3f)) return d;
    float id = 1.0f / d;
    *sx = c->halfW + c->focal * x1 * id;
    *sy = c->halfH - c->focal * y2 * id;
    return d;
}

/* a line in the world, half-width w (world units; its screen width follows each end's depth, averaged), clipped at
 * the near plane (depth 0.2) */
static void gl_line3(const GCam *c, float x0, float y0, float z0, float x1, float y1, float z1, float w,
                     const float c0[3], const float c1[3]) {
    const float NEAR = 0.2f;
    float ax, ay, bx, by;
    float da = gl_project(c, x0, y0, z0, &ax, &ay), db = gl_project(c, x1, y1, z1, &bx, &by);
    if (da <= NEAR && db <= NEAR) return;
    float ca[3] = { c0[0], c0[1], c0[2] }, cb[3] = { c1[0], c1[1], c1[2] };
    if (da <= NEAR || db <= NEAR) {                           /* cut at the near plane, in the world */
        float t = (NEAR + 0.01f - da) / (db - da);            /* depth is linear along a world line */
        float mx = x0 + (x1 - x0) * t, my = y0 + (y1 - y0) * t, mz = z0 + (z1 - z0) * t;
        float mc[3];
        for (int k = 0; k < 3; k++) mc[k] = c0[k] + (c1[k] - c0[k]) * t;
        float md = gl_project(c, mx, my, mz, da <= NEAR ? &ax : &bx, da <= NEAR ? &ay : &by);
        if (da <= NEAR) { da = md; memcpy(ca, mc, sizeof ca); }
        else { db = md; memcpy(cb, mc, sizeof cb); }
    }
    float r = w * c->focal * 0.5f * (1.0f / da + 1.0f / db);
    gl_line(ax, ay, bx, by, r, ca, cb);
}

/* a sprite in the world, radius w (world units) */
static void gl_sprite3(const GCam *c, float x, float y, float z, float w, float cr, float cg, float cb) {
    float sx, sy, d = gl_project(c, x, y, z, &sx, &sy);
    if (d <= 0.2f) return;
    gl_sprite(sx, sy, w * c->focal / d, cr, cg, cb);
}

/* ---------------------------------------------------------------------------------------- the final pass */
typedef struct {
    int use_bloom, trails, prefilled;
    uint32_t bgpix, d16;
    const uint16_t *U;
    uint16_t *acc16;
    const int *bx0;
    const uint8_t *bw8;
} GJob;

typedef struct {                   /* one thread's scratch */
    uint16_t L[GL_TILE * GL_TILE * 3];
    uint16_t bcell[3 * (GL_ROW_MAX / 4 + 2)];
    uint32_t row[GL_ROW_MAX];
    uint32_t left[GL_TILES_MAX];
} GScratch;

static GScratch gl_scr[2];
static GJob gl_job;
static int gl_next_band;
static int gl_threads = 2;
static int gl_worker_state = 0;    /* 0 not started, 1 running, -1 could not start */
static pthread_t gl_worker;
static pthread_mutex_t gl_mx = PTHREAD_MUTEX_INITIALIZER;
static pthread_cond_t gl_cv_go = PTHREAD_COND_INITIALIZER, gl_cv_done = PTHREAD_COND_INITIALIZER;
static unsigned gl_go_seq = 0, gl_done_seq = 0;
/* the worker's job: the final pass's bands, or a scene's gl_parallel_for */
static void (*gl_pf_fn)(void *ctx, int i0, int i1) = NULL;
static void *gl_pf_ctx = NULL;
static int gl_pf_n = 0, gl_pf_chunk = 1, gl_pf_next = 0;

static void gl_pf_run(void) {
    for (;;) {
        int c = __atomic_fetch_add(&gl_pf_next, 1, __ATOMIC_RELAXED);
        int i0 = c * gl_pf_chunk;
        if (i0 >= gl_pf_n)
            break;
        int i1 = i0 + gl_pf_chunk < gl_pf_n ? i0 + gl_pf_chunk : gl_pf_n;
        gl_pf_fn(gl_pf_ctx, i0, i1);
    }
}

static void gl_band_tiles(const GJob *J, int ty, GScratch *X) {
    const int TX = GF.TX, W = GF.W, H = GF.H, BW = GF.BW, BH = GF.BH, down = GF.down;
    const int y0 = ty * GL_TILE, y1 = y0 + GL_TILE < H ? y0 + GL_TILE : H;
    uint16_t *L = X->L;
    for (int tx = 0; tx < TX; tx++) {
        const int x0 = tx * GL_TILE, x1 = x0 + GL_TILE < W ? x0 + GL_TILE : W, t = ty * TX + tx;
        const int f = gl_tflag[t] & (J->use_bloom ? 3 : 1);
        if (!f && J->prefilled)
            continue;
        if (f & 1) {
            memset(L, 0, sizeof(uint16_t) * 3 * GL_TILE * (size_t)(y1 - y0));
            for (int e = gl_tstart[t]; e < gl_tstart[t + 1]; e++) {
                const GPrim *s = &gl_bin[e];
                const int xa = s->xa < x0 ? x0 : s->xa, xb = s->xb >= x1 ? x1 - 1 : s->xb;
                const int ya = s->ya < y0 ? y0 : s->ya, yb = s->yb >= y1 ? y1 - 1 : s->yb;
                if (xa > xb || ya > yb) continue;
                gl_splat(s, xa, xb, ya, yb, L, x0, y0, 3 * GL_TILE);
            }
        }
        int c_lo = (x0 >> 2) - 1, c_hi = ((x1 - 1) >> 2) + 1;
        c_lo = c_lo < 0 ? 0 : c_lo;
        c_hi = c_hi > BW - 1 ? BW - 1 : c_hi;
        for (int yy = y0; yy < y1; yy++) {
            uint32_t *orow = down == 1 ? (uint32_t *)(GF.dst + (size_t)yy * (size_t)GF.dpitch) : X->row;
            if (!f) {
                for (int xx = x0; xx < x1; xx++) orow[xx] = J->bgpix;
            } else {
                if (f & 2)
                    gl_bloom_row(X->bcell, J->U, BW, BH, yy, c_lo, c_hi);
                gl_tone_fx(orow, L + (size_t)(yy - y0) * (3 * GL_TILE), x0, X->bcell, J->bx0, J->bw8, BW, x0, x1, f,
                           GF.sh_r, GF.sh_g, GF.sh_b);
            }
            if (down > 1)
                for (int r = 0; r < down; r++) {
                    int oy = yy * down + r;
                    if (oy >= GF.dH) break;
                    uint32_t *o = (uint32_t *)(GF.dst + (size_t)oy * (size_t)GF.dpitch);
                    for (int xx = x0; xx < x1; xx++)
                        for (int k = 0; k < down; k++) {
                            int ox = xx * down + k;
                            if (ox < GF.dW) o[ox] = X->row[xx];
                        }
                }
        }
    }
}

static void gl_band_trails(const GJob *J, int ty, GScratch *X) {
    const int TX = GF.TX, W = GF.W, H = GF.H, BW = GF.BW, BH = GF.BH, down = GF.down;
    const int y0 = ty * GL_TILE, y1 = y0 + GL_TILE < H ? y0 + GL_TILE : H;
    const unsigned char *tf = gl_tflag + (size_t)ty * (size_t)TX;
    uint16_t *acc16 = J->acc16;
    uint32_t *left = X->left;
    int band_bloom = 0;
    for (int tx = 0; tx < TX; tx++) {
        const int x0 = tx * GL_TILE, x1 = x0 + GL_TILE < W ? x0 + GL_TILE : W, t = ty * TX + tx;
        for (int e = gl_tstart[t]; e < gl_tstart[t + 1]; e++) {
            const GPrim *s = &gl_bin[e];
            const int xa = s->xa < x0 ? x0 : s->xa, xb = s->xb >= x1 ? x1 - 1 : s->xb;
            const int ya = s->ya < y0 ? y0 : s->ya, yb = s->yb >= y1 ? y1 - 1 : s->yb;
            if (xa > xb || ya > yb) continue;
            gl_splat(s, xa, xb, ya, yb, acc16, 0, 0, 3 * W);
        }
        left[tx] = 0;
        band_bloom |= J->use_bloom ? tf[tx] & 2 : 0;
    }
    for (int yy = y0; yy < y1; yy++) {
        uint16_t *a = acc16 + 3 * (size_t)yy * (size_t)W;
        uint32_t *orow = down == 1 ? (uint32_t *)(GF.dst + (size_t)yy * (size_t)GF.dpitch) : X->row;
        if (band_bloom)
            gl_bloom_row(X->bcell, J->U, BW, BH, yy, 0, BW - 1);
        for (int tx = 0; tx < TX; tx++) {
            const int x0 = tx * GL_TILE, x1 = x0 + GL_TILE < W ? x0 + GL_TILE : W;
            const int f = tf[tx] & (J->use_bloom ? 3 : 1);
            if (!f) {
                if (!J->prefilled)
                    for (int xx = x0; xx < x1; xx++) orow[xx] = J->bgpix;
                continue;
            }
            gl_tone_fx(orow, a, 0, X->bcell, J->bx0, J->bw8, BW, x0, x1, f, GF.sh_r, GF.sh_g, GF.sh_b);
            if (f & 1) {
                uint32_t l = 0;
                const uint32_t d16 = J->d16;
                for (int k = 3 * x0; k < 3 * x1; k++) {
                    a[k] = (uint16_t)((a[k] * d16) >> 16);
                    l |= a[k];
                }
                left[tx] |= l;
            }
        }
        if (down == 1)
            continue;
        int oy0 = yy * down;
        if (oy0 >= GF.dH) break;
        uint32_t *o = (uint32_t *)(GF.dst + (size_t)oy0 * (size_t)GF.dpitch);
        int ox = 0;
        for (int xx = 0; xx < W && ox < GF.dW; xx++) {
            uint32_t v = X->row[xx];
            for (int k = 0; k < down && ox < GF.dW; k++) o[ox++] = v;
        }
        for (int r = 1; r < down && oy0 + r < GF.dH; r++)
            memcpy(GF.dst + (size_t)(oy0 + r) * (size_t)GF.dpitch, o, (size_t)GF.dW * 4u);
    }
    for (int tx = 0; tx < TX; tx++)
        gl_tres[ty * TX + tx] = left[tx] != 0;
}

static void gl_run_bands(const GJob *J, GScratch *X) {
    for (;;) {
        int ty = __atomic_fetch_add(&gl_next_band, 1, __ATOMIC_RELAXED);
        if (ty >= GF.TY)
            break;
        if (J->trails) gl_band_trails(J, ty, X);
        else gl_band_tiles(J, ty, X);
    }
}

static void *gl_worker_main(void *arg) {
    (void)arg;
    unsigned seen = 0;
    for (;;) {
        pthread_mutex_lock(&gl_mx);
        while (gl_go_seq == seen)
            pthread_cond_wait(&gl_cv_go, &gl_mx);
        seen = gl_go_seq;
        pthread_mutex_unlock(&gl_mx);
        if (gl_pf_fn) gl_pf_run();
        else gl_run_bands(&gl_job, &gl_scr[1]);
        pthread_mutex_lock(&gl_mx);
        gl_done_seq = seen;
        pthread_cond_signal(&gl_cv_done);
        pthread_mutex_unlock(&gl_mx);
    }
    return NULL;
}

static int gl_worker_up(void) {
    if (gl_threads == 2 && gl_worker_state == 0) {
        pthread_attr_t at;
        pthread_attr_init(&at);
        pthread_attr_setstacksize(&at, 256 * 1024);
        gl_worker_state = pthread_create(&gl_worker, &at, gl_worker_main, NULL) == 0 ? 1 : -1;
        pthread_attr_destroy(&at);
    }
    return gl_threads == 2 && gl_worker_state == 1;
}

/* hand the worker its job (set up before this) and do the same job here; returns when both are done. The mutex hands
 * the job over and the result back, so each side sees the other's writes */
static void gl_both(void (*here)(void)) {
    pthread_mutex_lock(&gl_mx);
    unsigned seq = ++gl_go_seq;
    pthread_cond_signal(&gl_cv_go);
    pthread_mutex_unlock(&gl_mx);
    here();
    pthread_mutex_lock(&gl_mx);
    while (gl_done_seq != seq)
        pthread_cond_wait(&gl_cv_done, &gl_mx);
    pthread_mutex_unlock(&gl_mx);
}

static void gl_bands_here(void) { gl_run_bands(&gl_job, &gl_scr[0]); }

static void gl_run_final(void) {
    __atomic_store_n(&gl_next_band, 0, __ATOMIC_RELAXED);
    gl_pf_fn = NULL;
    if (gl_worker_up() && GF.TY > 1) gl_both(gl_bands_here);
    else gl_run_bands(&gl_job, &gl_scr[0]);
}

/* a scene's per-element work on both cores: fn(ctx, i0, i1) for chunks of [0, n) (chunk elements each, taken in turn
 * by the engine's thread and the worker), returning when all are done. fn must only write what its elements own (no
 * gl_sprite / gl_line: those append to one list) - e.g. each element's position and colour into arrays, emitted as
 * primitives afterwards */
static void gl_parallel_for(void (*fn)(void *ctx, int i0, int i1), void *ctx, int n, int chunk) {
    if (n <= 0 || !fn) return;
    chunk = chunk < 1 ? 1 : chunk;
    if (!gl_worker_up() || n <= chunk) {
        fn(ctx, 0, n);
        return;
    }
    gl_pf_fn = fn;
    gl_pf_ctx = ctx;
    gl_pf_n = n;
    gl_pf_chunk = chunk;
    __atomic_store_n(&gl_pf_next, 0, __ATOMIC_RELAXED);
    gl_both(gl_pf_run);
    gl_pf_fn = NULL;
}

/* the bloom's blur (two separable box blurs of radius 2, in integers - exact, so empty cells stay 0), by rows so both
 * cores share it: the source's bright part into U, then U -> V across and V -> U down, twice */
typedef struct {
    const float *b0;
    uint16_t *U, *V;
    int BW, BH;
    float s;
} GBlur;

static void gl_blur_src(void *ctx, int y0, int y1) {
    const GBlur *B = (const GBlur *)ctx;
    const float thr = 0.35f, s = B->s;
    for (size_t k = 3 * (size_t)y0 * (size_t)B->BW; k < 3 * (size_t)y1 * (size_t)B->BW; k++)
        B->U[k] = (uint16_t)fminf(fmaxf(B->b0[k] - thr, 0.0f) * s, 65535.0f);
}

static void gl_blur_h(void *ctx, int y0, int y1) {
    const GBlur *B = (const GBlur *)ctx;
    const int BW = B->BW, RW = 3 * BW;
    for (int by = y0; by < y1; by++) {
        const uint16_t *restrict si = B->U + (size_t)by * (size_t)RW;
        uint16_t *restrict o = B->V + (size_t)by * (size_t)RW;
        for (int k = 6; k < RW - 6; k++)
            o[k] = (uint16_t)(((uint32_t)si[k - 6] + si[k - 3] + si[k] + si[k + 3] + si[k + 6]) / 5u);
        for (int bx = 0; bx < BW; bx++) {
            if (bx == 2 && BW - 2 > 2) bx = BW - 2;
            for (int c = 0; c < 3; c++) {
                uint32_t sum = 0;
                for (int j = -2; j <= 2; j++) {
                    int x = bx + j < 0 ? 0 : (bx + j >= BW ? BW - 1 : bx + j);
                    sum += si[3 * x + c];
                }
                o[3 * bx + c] = (uint16_t)(sum / 5u);
            }
        }
    }
}

static void gl_blur_v(void *ctx, int y0, int y1) {
    const GBlur *B = (const GBlur *)ctx;
    const int BH = B->BH, RW = 3 * B->BW;
    for (int by = y0; by < y1; by++) {
        const uint16_t *r[5];
        for (int j = -2; j <= 2; j++) {
            int y = by + j < 0 ? 0 : (by + j >= BH ? BH - 1 : by + j);
            r[j + 2] = B->V + (size_t)y * (size_t)RW;
        }
        uint16_t *restrict o = B->U + (size_t)by * (size_t)RW;
        for (int k = 0; k < RW; k++)
            o[k] = (uint16_t)(((uint32_t)r[0][k] + r[1][k] + r[2][k] + r[3][k] + r[4][k]) / 5u);
    }
}

/* the rest of the frame: bin, bloom (bloom_amt 0..~2: how much of the bright part glows), the tone curve at exposure
 * over bg (0..1 each), trails (decay 0 = none, else the light kept each frame, <= 0.9), into the screen. prefilled = 1:
 * the destination already holds exactly bg (the engine's fill), so dark tiles are skipped */
static void gl_end(float bloom_amt, float decay, float exposure, const float bgc[3], int prefilled) {
    if (!GF.ok) return;
    GF.ok = 0;
    double t0 = gl_now_ms();
    const int TX = GF.TX, NT = GF.NT, W = GF.W, H = GF.H, BW = GF.BW, BH = GF.BH;
    /* bin: a copy of each primitive in the bin of every tile its box touches */
    memset(gl_tstart, 0, sizeof(int) * (size_t)(NT + 1));
    size_t copies = 0;
    for (int i = 0; i < gl_n; i++) {
        const GPrim *s = &gl_list[i];
        const int tx0 = s->xa / GL_TILE, tx1 = s->xb / GL_TILE, ty0 = s->ya / GL_TILE, ty1 = s->yb / GL_TILE;
        for (int ty = ty0; ty <= ty1; ty++)
            for (int tx = tx0; tx <= tx1; tx++)
                gl_tstart[ty * TX + tx + 1]++;
        copies += (size_t)(tx1 - tx0 + 1) * (size_t)(ty1 - ty0 + 1);
    }
    if (copies > (size_t)gl_bin_cap) {
        size_t nc = copies + copies / 2 + 1024;
        GPrim *nb = (GPrim *)realloc(gl_bin, sizeof(GPrim) * nc);
        if (!nb) return;                                      /* (nothing drawn this frame) */
        gl_bin = nb;
        gl_bin_cap = (int)nc;
    }
    for (int t = 0; t < NT; t++) gl_tstart[t + 1] += gl_tstart[t];
    memcpy(gl_tfill, gl_tstart, sizeof(int) * (size_t)NT);
    for (int i = 0; i < gl_n; i++) {
        const GPrim *s = &gl_list[i];
        const int tx0 = s->xa / GL_TILE, tx1 = s->xb / GL_TILE, ty0 = s->ya / GL_TILE, ty1 = s->yb / GL_TILE;
        for (int ty = ty0; ty <= ty1; ty++)
            for (int tx = tx0; tx <= tx1; tx++)
                gl_bin[gl_tfill[ty * TX + tx]++] = *s;
    }
    /* trails: a persistent accumulator, cleared when they start (or on a new buffer / size) */
    decay = gl_clampf(decay, 0.0f, 0.9f);
    const int trails = decay > 0.0f;
    if (trails && (gl_acc_dirty || GF.acc != gl_acc_last || W != gl_acc_w || H != gl_acc_h)) {
        memset(GF.acc, 0, sizeof(float) * 3 * (size_t)W * (size_t)H);
        memset(gl_tres, 0, sizeof gl_tres);
    }
    gl_acc_last = GF.acc;
    gl_acc_w = W;
    gl_acc_h = H;
    gl_acc_dirty = !trails;
    for (int t = 0; t < NT; t++)
        gl_tflag[t] = (unsigned char)(gl_tstart[t + 1] > gl_tstart[t] || (trails && gl_tres[t]));
    double t1 = gl_now_ms();
    /* bloom: the bright part of the quarter-resolution source, blurred twice (separable box, radius 2), integers */
    bloom_amt = gl_clampf(bloom_amt, 0.0f, 4.0f);
    const int use_bloom = GF.bloom_on && bloom_amt > 0.0f;
    float *b0 = GF.bloom;
    uint16_t *U = (uint16_t *)(GF.bloom + 3 * (size_t)BW * (size_t)BH), *V = U + 3 * (size_t)BW * (size_t)BH;
    if (use_bloom) {
        GBlur bl = { b0, U, V, BW, BH, bloom_amt * GL_LFX };
        gl_parallel_for(gl_blur_src, &bl, BH, 8);             /* the bright part, as light x 256 */
        for (int pass = 0; pass < 2; pass++) {                /* two box blurs, rows split over both cores */
            gl_parallel_for(gl_blur_h, &bl, BH, 8);
            gl_parallel_for(gl_blur_v, &bl, BH, 8);
        }
        for (int by = 0; by < BH; by++) {                     /* the tiles the bloom reaches */
            int ya = by * 4 - 4, yb = by * 4 + 7;
            ya = ya < 0 ? 0 : ya;
            yb = yb >= H ? H - 1 : yb;
            for (int bx = 0; bx < BW; bx++) {
                const uint16_t *u = U + 3 * ((size_t)by * (size_t)BW + (size_t)bx);
                if (!(u[0] | u[1] | u[2])) continue;
                int xa = bx * 4 - 4, xb = bx * 4 + 7;
                xa = xa < 0 ? 0 : xa;
                xb = xb >= W ? W - 1 : xb;
                for (int ty = ya / GL_TILE; ty <= yb / GL_TILE; ty++)
                    for (int tx = xa / GL_TILE; tx <= xb / GL_TILE; tx++)
                        gl_tflag[ty * TX + tx] |= 2;
            }
        }
    }
    double t2 = gl_now_ms();
    /* the final pass */
    const float bgv[3] = { gl_clampf(bgc[0], 0.0f, 1.0f), gl_clampf(bgc[1], 0.0f, 1.0f), gl_clampf(bgc[2], 0.0f, 1.0f) };
    gl_tone_update(gl_clampf(exposure, 0.0f, 100.0f), bgv);
    static int bx0[GL_ROW_MAX];
    static uint8_t bw8[GL_ROW_MAX];
    if (use_bloom)
        for (int xx = 0; xx < W; xx++) {
            float fx = ((float)xx + 0.5f) * 0.25f - 0.5f;
            int x0 = (int)floorf(fx);
            float w = fx - (float)x0;
            if (x0 < 0) { x0 = 0; w = 0.0f; }
            if (x0 >= BW - 1) { x0 = BW - 1; w = 0.0f; }
            bx0[xx] = x0;
            bw8[xx] = (uint8_t)(w * 8.0f + 0.5f);
        }
    for (int t = 0; t < NT; t++) {
        int f = gl_tflag[t] & (use_bloom ? 3 : 1);
        gl_tiles[(f & 1) ? 0 : (f ? 1 : 2)] += 1.0;
    }
    GJob *J = &gl_job;
    J->use_bloom = use_bloom;
    J->trails = trails;
    J->prefilled = prefilled && GF.down == 1;
    J->bgpix = ((uint32_t)gl_tm8[0][0] << GF.sh_r) | ((uint32_t)gl_tm8[1][0] << GF.sh_g) |
               ((uint32_t)gl_tm8[2][0] << GF.sh_b);
    J->d16 = (uint32_t)(decay * 65536.0f);
    J->U = U;
    J->acc16 = (uint16_t *)GF.acc;
    J->bx0 = bx0;
    J->bw8 = bw8;
    gl_run_final();
    double t3 = gl_now_ms();
    gl_stage[0] += t1 - t0;
    gl_stage[1] += t2 - t1;
    gl_stage[2] += t3 - t2;
    gl_stage[3] += 1.0;
}

/* the next frame with trails starts from a clear accumulator (a mode shown again after a pause) */
static void gl_trails_reset(void) { gl_acc_dirty = 1; }

/* 1 or 2 threads for the final pass; the number in use */
static int gl_set_threads(int n) {
    gl_threads = n >= 2 ? 2 : 1;
    return gl_threads == 2 && gl_worker_state >= 0 ? 2 : 1;
}

/* mean ms per frame since the last call: bin, bloom, final pass; tiles lit, bloom only, dark; primitives last
 * frame, dropped (a full list); reset */
static void gl_stats(float out[8]) {
    double fr = gl_stage[3] > 0.0 ? gl_stage[3] : 1.0;
    out[0] = (float)(gl_stage[0] / fr);
    out[1] = (float)(gl_stage[1] / fr);
    out[2] = (float)(gl_stage[2] / fr);
    out[3] = (float)(gl_tiles[0] / fr);
    out[4] = (float)(gl_tiles[1] / fr);
    out[5] = (float)(gl_tiles[2] / fr);
    out[6] = (float)gl_n;
    out[7] = (float)gl_overflow;
    memset(gl_stage, 0, sizeof gl_stage);
    memset(gl_tiles, 0, sizeof gl_tiles);
}

/* ---------------------------------------------------------------------------------------- measuring on the device */
#if defined(__ARM_NEON) || defined(__ARM_NEON__)
#include <arm_neon.h>
#define GL_HAVE_NEON 1
#else
#define GL_HAVE_NEON 0
#endif

/* the sprite splat on an RGBX buffer (4 x u16 a pixel), the channels as one NEON vector: the candidate for the tile
 * buffers (the benchmark compares it with gl_splat_sprite's three scalar channels) */
static inline void gl_splat_sprite4(const GPrim *s, int xa, int xb, int ya, int yb, uint16_t *buf, int bx, int by,
                                    int stride) {
    const float QS = 1048576.0f;
    const float is = s->inv_r;
    const float dx0 = ((float)xa + 0.5f - s->x0) * is;
    const int32_t qx0 = (int32_t)(dx0 * dx0 * QS);
    const int32_t d10 = (int32_t)((2.0f * dx0 * is + is * is) * QS);
    const int32_t d2 = (int32_t)(2.0f * is * is * QS);
#if GL_HAVE_NEON
    const uint16x4_t col = { s->c0[0], s->c0[1], s->c0[2], 0 };
#endif
    for (int y = ya; y <= yb; y++) {
        float dy = ((float)y + 0.5f - s->y0) * is;
        int32_t qy = (int32_t)(dy * dy * QS);
        if ((qy >> 10) >= GL_SPR2_N) continue;
        uint16_t *p = buf + (size_t)(y - by) * (size_t)stride + 4 * (size_t)(xa - bx);
        int32_t q = qx0 + qy, d1 = d10;
        for (int xx = xa; xx <= xb; xx++, p += 4) {
            int32_t qi = q >> 10;
            q += d1;
            d1 += d2;
            qi &= ~(qi >> 31);
            if (qi >= GL_SPR2_N) continue;
            uint32_t w = gl_prof16[qi];
#if GL_HAVE_NEON
            uint16x4_t v = vld1_u16(p);
            vst1_u16(p, vqadd_u16(v, vshrn_n_u32(vmull_n_u16(col, (uint16_t)w), 16)));
#else
            p[0] = gl_sat(p[0] + ((s->c0[0] * w) >> 16));
            p[1] = gl_sat(p[1] + ((s->c0[1] * w) >> 16));
            p[2] = gl_sat(p[2] + ((s->c0[2] * w) >> 16));
#endif
        }
    }
}

/* ns per pixel on this machine: which 0 = sprites of radius r splatted into a tile (RGB, as now), 1 = the same into an
 * RGBX tile (gl_splat_sprite4), 2 = the tone pass over lit tiles (n tiles), 3 = lines of half-width r; n repeats */
static float gl_bench(int which, int n, float r) {
    static uint16_t L[GL_TILE * GL_TILE * 4];
    static uint32_t out[GL_TILE * GL_TILE];
    if (!gl_luts_ready) gl_luts_init();
    const float bgz[3] = { 0.02f, 0.02f, 0.05f };
    gl_tone_update(1.0f, bgz);
    memset(L, 0, sizeof L);
    r = gl_clampf(r, 0.75f, 12.0f);
    n = n < 1 ? 1 : (n > 1000000 ? 1000000 : n);
    uint32_t rs = 99u;
    double px = 0.0;
    double t0 = gl_now_ms();
    for (int i = 0; i < n; i++) {
        rs ^= rs << 13; rs ^= rs >> 17; rs ^= rs << 5;
        float x = (float)(rs & 31) + (float)((rs >> 5) & 7) * 0.125f, y = (float)((rs >> 8) & 31) + 0.37f;
        GPrim p;
        memset(&p, 0, sizeof p);
        p.x0 = x; p.y0 = y; p.inv_r = 1.0f / r;
        p.c0[0] = 200; p.c0[1] = 120; p.c0[2] = 60; p.c1[0] = 60; p.c1[1] = 120; p.c1[2] = 200;
        int R = (int)ceilf(r);
        int xa = (int)floorf(x) - R, xb = (int)floorf(x) + R, ya = (int)floorf(y) - R, yb = (int)floorf(y) + R;
        xa = xa < 0 ? 0 : xa; ya = ya < 0 ? 0 : ya; xb = xb > GL_TILE - 1 ? GL_TILE - 1 : xb; yb = yb > GL_TILE - 1 ? GL_TILE - 1 : yb;
        if (which == 0) {
            gl_splat_sprite(&p, xa, xb, ya, yb, L, 0, 0, 3 * GL_TILE);
            px += (double)(xb - xa + 1) * (double)(yb - ya + 1);
        } else if (which == 1) {
            gl_splat_sprite4(&p, xa, xb, ya, yb, L, 0, 0, 4 * GL_TILE);
            px += (double)(xb - xa + 1) * (double)(yb - ya + 1);
        } else if (which == 2) {
            for (int yy = 0; yy < GL_TILE; yy++)
                gl_tone_gen(out + yy * GL_TILE, L + yy * 3 * GL_TILE, 0, NULL, NULL, NULL, 0, GL_TILE, 1, 16, 8, 0);
            px += GL_TILE * GL_TILE;
            L[i % (GL_TILE * GL_TILE * 3)] += 7;
        } else {
            p.kind = 1;
            p.ux = 0.8f; p.uy = 0.6f; p.len = 24.0f; p.x0 = x * 0.2f; p.y0 = y * 0.3f;
            int lxa = 0, lxb = GL_TILE - 1, lya = 0, lyb = GL_TILE - 1;
            gl_splat_line(&p, lxa, lxb, lya, lyb, L, 0, 0, 3 * GL_TILE);
            px += 24.0 * (2.0 * r + 3.0);
        }
    }
    double el = gl_now_ms() - t0;
    volatile uint32_t sink = out[5] + L[77];
    (void)sink;
    return (float)(el * 1.0e6 / (px > 0.0 ? px : 1.0));
}

/* ---------------------------------------------------------------------------------------- a stroke font */
/* glyphs on a 4 x 6 grid, each a list of segments x0 y0 x1 y1 (y up), 0xFF ends it: digits, capitals, a dot, a dash */
static const unsigned char *gl_glyph(char ch) {
    static const unsigned char G_0[] = { 0,0,4,0, 4,0,4,6, 4,6,0,6, 0,6,0,0, 0,0,4,6, 0xFF };
    static const unsigned char G_1[] = { 2,0,2,6, 2,6,1,5, 1,0,3,0, 0xFF };
    static const unsigned char G_2[] = { 0,6,4,6, 4,6,4,3, 4,3,0,3, 0,3,0,0, 0,0,4,0, 0xFF };
    static const unsigned char G_3[] = { 0,6,4,6, 4,6,4,0, 4,0,0,0, 1,3,4,3, 0xFF };
    static const unsigned char G_4[] = { 0,6,0,3, 0,3,4,3, 4,6,4,0, 0xFF };
    static const unsigned char G_5[] = { 4,6,0,6, 0,6,0,3, 0,3,4,3, 4,3,4,0, 4,0,0,0, 0xFF };
    static const unsigned char G_6[] = { 4,6,0,6, 0,6,0,0, 0,0,4,0, 4,0,4,3, 4,3,0,3, 0xFF };
    static const unsigned char G_7[] = { 0,6,4,6, 4,6,1,0, 0xFF };
    static const unsigned char G_8[] = { 0,0,4,0, 4,0,4,6, 4,6,0,6, 0,6,0,0, 0,3,4,3, 0xFF };
    static const unsigned char G_9[] = { 4,3,0,3, 0,3,0,6, 0,6,4,6, 4,6,4,0, 4,0,0,0, 0xFF };
    static const unsigned char G_A[] = { 0,0,0,4, 0,4,2,6, 2,6,4,4, 4,4,4,0, 0,3,4,3, 0xFF };
    static const unsigned char G_B[] = { 0,0,0,6, 0,6,3,6, 3,6,4,5, 4,5,3,3, 3,3,0,3, 3,3,4,2, 4,2,4,1, 4,1,3,0, 3,0,0,0, 0xFF };
    static const unsigned char G_C[] = { 4,6,0,6, 0,6,0,0, 0,0,4,0, 0xFF };
    static const unsigned char G_D[] = { 0,0,0,6, 0,6,3,6, 3,6,4,5, 4,5,4,1, 4,1,3,0, 3,0,0,0, 0xFF };
    static const unsigned char G_E[] = { 4,6,0,6, 0,6,0,0, 0,0,4,0, 0,3,3,3, 0xFF };
    static const unsigned char G_F[] = { 4,6,0,6, 0,6,0,0, 0,3,3,3, 0xFF };
    static const unsigned char G_H[] = { 0,0,0,6, 4,0,4,6, 0,3,4,3, 0xFF };
    static const unsigned char G_I[] = { 1,6,3,6, 2,6,2,0, 1,0,3,0, 0xFF };
    static const unsigned char G_J[] = { 4,6,4,1, 4,1,3,0, 3,0,1,0, 1,0,0,1, 0xFF };
    static const unsigned char G_K[] = { 0,0,0,6, 0,3,4,6, 0,3,4,0, 0xFF };
    static const unsigned char G_L[] = { 0,6,0,0, 0,0,4,0, 0xFF };
    static const unsigned char G_M[] = { 0,0,0,6, 0,6,2,3, 2,3,4,6, 4,6,4,0, 0xFF };
    static const unsigned char G_N[] = { 0,0,0,6, 0,6,4,0, 4,0,4,6, 0xFF };
    static const unsigned char G_O[] = { 0,0,4,0, 4,0,4,6, 4,6,0,6, 0,6,0,0, 0xFF };
    static const unsigned char G_P[] = { 0,0,0,6, 0,6,4,6, 4,6,4,3, 4,3,0,3, 0xFF };
    static const unsigned char G_Q[] = { 0,0,4,0, 4,0,4,6, 4,6,0,6, 0,6,0,0, 2,2,4,0, 0xFF };
    static const unsigned char G_R[] = { 0,0,0,6, 0,6,4,6, 4,6,4,3, 4,3,0,3, 1,3,4,0, 0xFF };
    static const unsigned char G_S[] = { 4,6,0,6, 0,6,0,3, 0,3,4,3, 4,3,4,0, 4,0,0,0, 0xFF };
    static const unsigned char G_T[] = { 0,6,4,6, 2,6,2,0, 0xFF };
    static const unsigned char G_U[] = { 0,6,0,0, 0,0,4,0, 4,0,4,6, 0xFF };
    static const unsigned char G_V[] = { 0,6,2,0, 2,0,4,6, 0xFF };
    static const unsigned char G_W[] = { 0,6,1,0, 1,0,2,3, 2,3,3,0, 3,0,4,6, 0xFF };
    static const unsigned char G_X[] = { 0,0,4,6, 0,6,4,0, 0xFF };
    static const unsigned char G_Y[] = { 0,6,2,3, 4,6,2,3, 2,3,2,0, 0xFF };
    static const unsigned char G_DOT[] = { 2,0,2,1, 0xFF };
    static const unsigned char G_DASH[] = { 1,3,3,3, 0xFF };
    switch (ch) {
    case '0': return G_0; case '1': return G_1; case '2': return G_2; case '3': return G_3; case '4': return G_4;
    case '5': return G_5; case '6': return G_6; case '7': return G_7; case '8': return G_8; case '9': return G_9;
    case 'A': return G_A; case 'B': return G_B; case 'C': return G_C; case 'D': return G_D; case 'E': return G_E;
    case 'F': return G_F; case 'H': return G_H; case 'I': return G_I; case 'J': return G_J; case 'K': return G_K;
    case 'L': return G_L; case 'M': return G_M; case 'N': return G_N; case 'O': return G_O; case 'P': return G_P;
    case 'Q': return G_Q; case 'R': return G_R; case 'S': return G_S; case 'T': return G_T; case 'U': return G_U;
    case 'V': return G_V; case 'W': return G_W; case 'X': return G_X; case 'Y': return G_Y;
    case '.': return G_DOT; case '-': return G_DASH;
    default: return NULL;
    }
}

/* text in the world: from o along the unit vector ax, glyphs up along ay, h tall; half-width w (world units) */
static void gl_text3(const GCam *c, const char *s, const float o[3], const float ax[3], const float ay[3], float h,
                     float w, const float col[3]) {
    float k = h / 6.0f, adv = 5.2f * k;
    for (int i = 0; s[i]; i++) {
        const unsigned char *g = gl_glyph(s[i]);
        if (!g) continue;
        float bx = o[0] + ax[0] * adv * (float)i, by = o[1] + ax[1] * adv * (float)i, bz = o[2] + ax[2] * adv * (float)i;
        for (int j = 0; g[j] != 0xFF; j += 4) {
            float u0 = g[j] * k, v0 = g[j + 1] * k, u1 = g[j + 2] * k, v1 = g[j + 3] * k;
            gl_line3(c, bx + ax[0] * u0 + ay[0] * v0, by + ax[1] * u0 + ay[1] * v0, bz + ax[2] * u0 + ay[2] * v0,
                     bx + ax[0] * u1 + ay[0] * v1, by + ax[1] * u1 + ay[1] * v1, bz + ax[2] * u1 + ay[2] * v1, w, col, col);
        }
    }
}

/* ---------------------------------------------------------------------------------------- small maths for scenes */
static inline float gl_fract(float x) { return x - floorf(x); }

/* sin to ~1e-3 without libm (the A53 pays ~100 cycles for sinf) */
static inline float gl_sin(float x) {
    float t = x * 0.15915494f;
    t -= floorf(t + 0.5f);
    float y = 8.0f * t - 16.0f * t * fabsf(t);
    return 0.225f * (y * fabsf(y) - y) + y;
}

static inline float gl_cos(float x) { return gl_sin(x + 1.5707963f); }

/* THREE.Color.setHSL, branch-free */
static inline void gl_hsl(float h, float s, float l, float *out) {
    h = gl_fract(h);
    s = gl_c01(s);
    l = gl_c01(l);
    float q = l + s * fminf(l, 1.0f - l), p = 2.0f * l - q;
    for (int c = 0; c < 3; c++) {
        float t = gl_fract(h + (1.0f - (float)c) * (1.0f / 3.0f));
        out[c] = p + (q - p) * gl_c01(fminf(6.0f * t, 4.0f - 6.0f * t));
    }
}

static uint32_t gl_rng_state = 12345u;
static inline float gl_rnd(void) {                          /* xorshift32, [0, 1) */
    gl_rng_state ^= gl_rng_state << 13;
    gl_rng_state ^= gl_rng_state >> 17;
    gl_rng_state ^= gl_rng_state << 5;
    return (float)(gl_rng_state >> 8) * (1.0f / 16777216.0f);
}

static inline uint32_t gl_hash(uint32_t x) {
    x ^= x >> 16; x *= 0x7feb352du; x ^= x >> 15; x *= 0x846ca68bu; x ^= x >> 16;
    return x;
}

/* 1D value noise in [-1, 1], smooth */
static inline float gl_noise1(float x, uint32_t seed) {
    float fx = floorf(x);
    if (!(fabsf(fx) < 1.0e6f)) return 0.0f;
    int i = (int)fx;
    float t = x - fx, s = t * t * (3.0f - 2.0f * t);
    float a = (float)gl_hash((uint32_t)i * 2654435761u ^ seed) * (1.0f / 4294967296.0f);
    float b = (float)gl_hash((uint32_t)(i + 1) * 2654435761u ^ seed) * (1.0f / 4294967296.0f);
    return (a + (b - a) * s) * 2.0f - 1.0f;
}

#endif /* GLOW_H */
