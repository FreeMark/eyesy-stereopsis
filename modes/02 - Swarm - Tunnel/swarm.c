/* swarm.c - native renderer for the EYESY's Swarm modes (01 - Swarm - Ribbon ..): free.vet's drone-swarm visualizer
 * (web-swarm-sim/src/lightshow: VisualizerField.tsx, drone.vert/.frag.glsl, source.ts's cymatics) ported to
 * the EYESY's CPU.
 *
 * The mode compiles this file ON the EYESY the first time it runs, caches the .so in its folder, and calls
 * sw_frame() once per frame through ctypes. It writes straight into the screen surface's pixel memory, so
 * every argument is validated, every write is bounds-checked against the caller's pitch/height, and float
 * values only become indices after explicit range checks (no -ffast-math: it would delete the NaN guards).
 *
 * Per frame:
 *  1. layout: every drone's target position + colour for the current style, from the audio (stereopsis's
 *     analysis: a Web Audio-identical 1024-bin spectrum, the waveform, level/bass/mid/treble, the kick
 *     envelope). Styles as on free.vet: corona (radial spectrum sun), ribbon (the waveform on a ring),
 *     terrain (scrolling spectrogram), orb (spectrum-pushed Fibonacci sphere), nebula (value-noise cloud
 *     breathing with the bass), tunnel (rings shaped by the bass flying past), plus cymatics (drones as
 *     a Chladni plate vibrating as a standing wave (version 3; it was sand on the nodal lines); each kick steps
 *     the plate up its mode ladder).
 *  2. motion: drones ease toward their targets (free.vet's fast ease); a style change is a staggered
 *     smootherstep morph (each drone on its own clock) with a transition effect (explode, implode,
 *     spiral, fade, fountain - drone.vert.glsl's presets); Trigger bursts the swarm outward.
 *  3. render: soft round LEDs (free.vet's sprite: hot core + halo, additive), sized by perspective and
 *     swelling on beats and energy, treble twinkle, bass swell + spin; projected, sorted into 32x32
 *     tiles (the accumulator block stays in L1), splatted, then a quarter-resolution bloom, and one fused
 *     pass: tone map (1 - e^-x) over the background + trail decay + block upscale into the screen.
 *
 * Version 2: one build of this file serves every mode of the family - the Swarm Visualizer (all styles, knob 1
 * picks; retired 2026-09-26) and the single-figure modes, whose knob 1 is the camera distance (the tunnel's: a
 * horizontal turn about its middle). They load the same library, so they share its state: switching between them
 * morphs the swarm from one figure into the next. The mode that draws first after the library loads calls sw_init;
 * the rest check sw_ready (always on the engine's main thread, as sw_frame).
 * Version 3 (2026-09-30, Free's walkthrough): per-figure dials, P_PULSE .. P_SPAN, whose neutral values give version
 * 2's picture: the beat's swell and the bass spin can be turned down (the terrain holds still), a burst's reach set
 * (the nebula's follows the camera distance), the figure scaled while the Trigger is held (P_HOLD, P_SMOOTH: the mode
 * computes the breath), the ribbon's wave height, the orb tumbling about the horizontal axis, the tunnel lit by a share
 * of its drones (evenly spread: fewer rings, fewer drones a ring; the others fade out), and cymatics is free.vet's
 * PLATE: a grid of drones vibrating as a standing wave, its size in mode lengths a dial (the sand version is gone).
 */
#include <math.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <pthread.h>

static double g_stage[6];          /* ms spent per stage since the last sw_debug: layout+motion, project,
                                      clear+splat, bloom, final pass, frames */
static double g_tiles[3];          /* tiles per frame: lit (accumulator), bloom only, dark */

static double now_ms(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return ts.tv_sec * 1000.0 + ts.tv_nsec / 1.0e6;
}

#define SW_VERSION 3
#define MAXN 8192
#define NSTYLE 7
#define TUN_RINGS 64
#define TER_MAX 96                 /* terrain grid side limit (side = ceil(sqrt(n))) */
#define MAXR 24                    /* max sprite radius, accumulator pixels */
#define SPR2_N 1024                /* the sprite profile, by squared distance (the splat loop's LUT) */
#define ROW_MAX 8192
#define TILE 32
#define TILES_MAX 4096
#define TAU 6.283185307179586f
#define PI_F 3.14159265358979f

enum { ST_CORONA, ST_RIBBON, ST_TERRAIN, ST_ORB, ST_NEBULA, ST_TUNNEL, ST_CYMATICS };

enum {
    P_DT, P_STYLE, P_ORBIT, P_GLOW, P_HUE, P_BG_R, P_BG_G, P_BG_B,
    P_LEVEL, P_BASS, P_MID, P_TREBLE, P_BEAT, P_BEAT_EDGE, P_TRIG,
    P_SIZE, P_EXPOSURE, P_BRIGHT, P_FOV,
    P_PREFILLED,                   /* 1 = the destination already holds the background colour (the engine
                                      fills it before draw() when Persist is off): dark tiles are skipped */
    P_CAMD_MUL,                    /* camera distance x this (1 = the style's own; the single-style modes' knob 1) */
    P_TURN,                        /* radians: the view turned about the style's pivot (the tunnel mode's knob 1) */
    P_ROLL,                        /* -1..1: the tunnel's rings roll about its bore, one way or the other */
    P_CAP,                         /* the largest sprite, px across at 360 lines (16; the single-style modes lower it
                                      as the camera comes close, where near drones would all reach it) */
    /* version 3: the single-figure modes' dials (in brackets the neutral value: version 2's behaviour) */
    P_PULSE,                       /* [1] how much the beat and the loudness swell the swarm; 0 = it holds still */
    P_SPIN,                        /* [1] the bass spinning the camera; 0 = only the knob turns it */
    P_BURST_AMP,                   /* [1] how far a burst throws the drones */
    P_HOLD,                        /* [1] the figure's scale (the Trigger held: the mode breathes it) */
    P_SMOOTH,                      /* [0] 0..1: the music's shaping smoothed away (a clean ring, a round orb) */
    P_AMP,                         /* [1] the ribbon's wave height */
    P_TUMBLE,                      /* [0] -1..1: the figure turns about the horizontal axis (the orb's knob 2) */
    P_FRAC,                        /* [1] 0.1..1: the share of the drones lit, evenly spread (the tunnel's knob 2) */
    P_SPAN,                        /* [1] the plate's size in its modes' lengths: more nodes (cymatics' knob 1) */
    P_COUNT
};

/* a projected sprite, as the splat reads it: centre, 1 / radius, light at the profile's peak (fixed point, x 256)
 * and its bounding box on the render, clipped (28 bytes: the tiles read them in runs, straight from the bins) */
typedef struct { float sx, sy, inv_r; uint16_t fr, fg, fb; int16_t xa, xb, ya, yb; } Spr;

/* light in the accumulators is fixed point: 1.0 = 256 in a uint16 (so up to 256 - the tone curve is flat long
 * before). The A53 is in-order with a 64-bit NEON unit: a float per pixel means 4-cycle latency chains and a
 * transfer to the core for every table index; integers issue two a cycle with 1-3 cycle latencies */
#define LFX 256.0f
#define TM8_N 4096                 /* tone tables: light 0 .. 16 in steps of 1/256, a byte per channel */

static struct {
    int n, ready;
    float pos[MAXN * 3], tgt[MAXN * 3], start[MAXN * 3];
    float col[MAXN * 3], tcol[MAXN * 3], scol[MAXN * 3];
    float seed[MAXN], dirs[MAXN * 3], anch[MAXN * 3];
    int side;
    float hist[TER_MAX * TER_MAX];
    int row_ptr;
    float hop_t;
    float tun_hist[TUN_RINGS];
    int tun_ptr;
    float tun_hop, tun_phase;
    int style, have_style, auto_mode, auto_beats;
    float auto_t;
    float morph;
    int morphing, trans_kind;
    float time, yaw, pitch, spin, burst, yaw_w, camd;
    float pivz, turn, roll;       /* the turn's pivot on the z axis (eased per style), the turn (eased), the roll */
    float tumble;                 /* the turn about the horizontal axis (P_TUMBLE), radians */
    float vis[MAXN], tvis[MAXN];  /* each drone's light (0..1, eased toward tvis: 0 = not lit in this figure) */
    int cym_idx;                  /* the plate's mode on the ladder */
    float cym_m, cym_n, cym_last; /* its mode numbers, eased toward the ladder's (figures melt); the last step's time */
    short bin_a[MAXN];            /* each drone's spectrum bin (corona, orb: free.vet's freqAt to 8.4 kHz) */
    short bin_t[TER_MAX];         /* each terrain column's bin (to 7.2 kHz) */
    float bins_hz;                /* the bin width + count those were made for */
    int bins_nfft;
    int neb_cell[MAXN * 3];       /* nebula: each drone's three noises, the lattice cell along the moving axis ... */
    float neb_e[MAXN * 6];        /* ... and the noise at that cell's two faces (see neb_noise) */
} S;

static float g_spr2[SPR2_N];
static uint16_t g_prof16[SPR2_N];  /* the same profile, its peak = 65535 */
static float g_prof_max;           /* the profile's peak (1.5) */
static float g_spr_energy;         /* the profile's integral over the unit disc (light per sprite / r^2) */
static int g_luts = 0;
static float g_t8[TM8_N];          /* the tone curve 1 - e^-(E light) for g_t8_E */
static float g_t8_E = -1.0f;
static uint8_t g_tm8[3][TM8_N];    /* ... over the background: a byte per channel, for g_tm8_bg */
static float g_tm8_bg[3] = { -1.0f, -1.0f, -1.0f };
static unsigned char g_tflag[TILES_MAX];   /* per 32x32 tile this frame: 1 = sprites wrote the accumulator,
                                              2 = bloom reaches it */
static const float *g_acc_last = NULL;    /* the accumulator trails used last frame, at this size */
static int g_acc_w = 0, g_acc_h = 0;
static int g_acc_dirty = 1;               /* 1 = it does not hold last frame's trails (a new start, no trails) */
static unsigned char g_tres[TILES_MAX];   /* trails: 1 = light is left in the tile's part of the accumulator */
static Spr *g_spr_list = NULL;            /* this frame's sprites, in drone order */
static Spr *g_bin = NULL;                 /* ... binned: a copy in every tile its box touches, tile by tile */
static int g_cap = 0;                     /* drones the two have room for (the bins: 9 copies a drone) */
static int g_tstart[TILES_MAX + 1];       /* where each tile's bin starts (g_tstart[NT] = all copies) */
static int g_tfill[TILES_MAX];

int sw_version(void) { return SW_VERSION; }
int sw_param_count(void) { return P_COUNT; }
int sw_max_drones(void) { return MAXN; }

/* the drone count sw_init set up, 0 before it: modes sharing this library init it only when this differs */
int sw_ready(void) { return S.ready ? S.n : 0; }

/* the next frame with trails starts from a clear accumulator (a mode shown again after a pause: without this, the
 * light it left there minutes ago would flash back for a moment) */
void sw_trails_reset(void) { g_acc_dirty = 1; }

static inline float clampf(float v, float lo, float hi) {
    if (!(v == v)) return lo;                                 /* NaN -> lo */
    return v < lo ? lo : (v > hi ? hi : v);
}

static void luts_init(void) {
    /* free.vet's drone.frag, by the squared distance q = u^2 (u = r / radius): core = smoothstep(0.5, 0.0,
     * r/2), halo = core^2.4, light = colour x core x (0.35 + 1.15 halo) (additive, alpha = core) */
    double e = 0.0;
    for (int i = 0; i < SPR2_N; i++) {
        float u = sqrtf(((float)i + 0.5f) / (float)SPR2_N), t = 1.0f - u;
        float core = t * t * (3.0f - 2.0f * t);
        g_spr2[i] = core * (0.35f + powf(core, 2.4f) * 1.15f);
        e += g_spr2[i];                                       /* uniform in u^2 = uniform in area */
    }
    g_spr_energy = (float)(3.14159265358979 * e / SPR2_N);
    g_prof_max = 0.0f;
    for (int i = 0; i < SPR2_N; i++) g_prof_max = g_spr2[i] > g_prof_max ? g_spr2[i] : g_prof_max;
    for (int i = 0; i < SPR2_N; i++) g_prof16[i] = (uint16_t)(g_spr2[i] / g_prof_max * 65535.0f + 0.5f);
    g_luts = 1;
}

/* ---------------------------------------------------------------------------------------- the tone curve */
/* light -> pixel: channel = 255 (bg + (1 - bg)(1 - e^-(E light))) + 0.5, in byte tables over the fixed-point
 * light (4096 entries, steps of 1/256). The curve is rebuilt when E changes, the tables when the background
 * does (both rarely: E is constant, the background follows knob 5) */
static void tm8_update(float E, const float bg[3]) {
    if (E != g_t8_E) {
        for (int i = 0; i < TM8_N; i++) g_t8[i] = 1.0f - expf(-E * (float)i / LFX);
        g_t8_E = E;
        g_tm8_bg[0] = -1.0f;                                     /* force the tables */
    }
    if (bg[0] != g_tm8_bg[0] || bg[1] != g_tm8_bg[1] || bg[2] != g_tm8_bg[2]) {
        for (int c = 0; c < 3; c++) {
            for (int i = 0; i < TM8_N; i++)
                g_tm8[c][i] = (uint8_t)(255.0f * (bg[c] + (1.0f - bg[c]) * g_t8[i]) + 0.5f);
            g_tm8_bg[c] = bg[c];
        }
    }
}

/* n pixels: light from a (if use_a) + bloom already expanded per pixel in bl (if use_b), through the tone tables,
 * packed at the given shifts. Inlined with constants (the EYESY's XRGB is 16/8/0), so the loop keeps the tables
 * and shifts in registers: the generic version reloaded six of them from the stack every pixel on the A53 */
static inline __attribute__((always_inline)) void tone_run(uint32_t *restrict out, const uint16_t *restrict a,
                                                           const uint16_t *restrict bl, int n, int use_a, int use_b,
                                                           int sr, int sg, int sb) {
    const uint8_t *tr = g_tm8[0], *tg = g_tm8[1], *tb = g_tm8[2];
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
        r = r < TM8_N - 1 ? r : TM8_N - 1;
        g = g < TM8_N - 1 ? g : TM8_N - 1;
        b = b < TM8_N - 1 ? b : TM8_N - 1;
        out[i] = ((uint32_t)tr[r] << sr) | ((uint32_t)tg[g] << sg) | ((uint32_t)tb[b] << sb);
    }
}

/* groups of 4 pixels with light AND bloom on the XRGB layout, fused: pixels 4j .. 4j+3 read bloom cells j-1, j, j+1
 * once and weigh them 5, 7, 1, 3 eighths (exactly what bx0 / bw8 say for those pixels, so the result is bit-for-
 * bit the general path's); for 1 <= j <= BW - 2. The tunnel lights the whole screen on this path */
static void tone_ab4(uint32_t *out, const uint16_t *a, const uint16_t *bc, int j0, int groups) {
    const uint8_t *tr = g_tm8[0], *tg = g_tm8[1], *tb = g_tm8[2];
    const uint16_t *c = bc + 3 * (j0 - 1);
    for (int gi = 0; gi < groups; gi++, a += 12, out += 4, c += 3) {
        const uint32_t L0 = c[0], L1 = c[1], L2 = c[2], M0 = c[3], M1 = c[4], M2 = c[5], R0 = c[6], R1 = c[7], R2 = c[8];
#define SW_PIX(i, br, bgr, bbl)                                                                         \
        do {                                                                                            \
            uint32_t r_ = a[3 * (i)] + (br), g_ = a[3 * (i) + 1] + (bgr), b_ = a[3 * (i) + 2] + (bbl);  \
            r_ = r_ < TM8_N - 1 ? r_ : TM8_N - 1;                                                       \
            g_ = g_ < TM8_N - 1 ? g_ : TM8_N - 1;                                                       \
            b_ = b_ < TM8_N - 1 ? b_ : TM8_N - 1;                                                       \
            out[i] = ((uint32_t)tr[r_] << 16) | ((uint32_t)tg[g_] << 8) | (uint32_t)tb[b_];            \
        } while (0)
        SW_PIX(0, (3 * L0 + 5 * M0) >> 3, (3 * L1 + 5 * M1) >> 3, (3 * L2 + 5 * M2) >> 3);
        SW_PIX(1, (L0 + 7 * M0) >> 3, (L1 + 7 * M1) >> 3, (L2 + 7 * M2) >> 3);
        SW_PIX(2, (7 * M0 + R0) >> 3, (7 * M1 + R1) >> 3, (7 * M2 + R2) >> 3);
        SW_PIX(3, (5 * M0 + 3 * R0) >> 3, (5 * M1 + 3 * R1) >> 3, (5 * M2 + 3 * R2) >> 3);
#undef SW_PIX
    }
}

/* one run [x0, x1) of a row (x1 - x0 <= TILE): f = 1 the accumulator (fixed-point light, its first pixel at column
 * ax), 2 the bloom (fixed-point cells bc, 4x bilinear by bx0 / bw8 - the weights are exact eighths), 3 both. The
 * bloom is expanded to per-pixel values first, then one specialised loop per case. All integer */
static void tone_gen(uint32_t *orow, const uint16_t *a, int ax, const uint16_t *bc, const int *bx0,
                     const uint8_t *bw8, int x0, int x1, int f, int sh_r, int sh_g, int sh_b) {
    uint16_t bl[3 * TILE];
    const int n = x1 - x0;
    if (n <= 0 || n > TILE)
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
        if (f == 1) tone_run(out, ap, NULL, n, 1, 0, 16, 8, 0);
        else if (f == 2) tone_run(out, NULL, bl, n, 0, 1, 16, 8, 0);
        else tone_run(out, ap, bl, n, 1, 1, 16, 8, 0);
    } else {
        tone_run(out, ap, bl, n, f & 1, (f & 2) >> 1, sh_r, sh_g, sh_b);
    }
}

/* the same, but with light and bloom on XRGB the 4-aligned middle of the run goes through tone_ab4 */
static void tone_fx(uint32_t *orow, const uint16_t *a, int ax, const uint16_t *bc, const int *bx0,
                    const uint8_t *bw8, int BW, int x0, int x1, int f, int sh_r, int sh_g, int sh_b) {
    if (f == 3 && sh_r == 16 && sh_g == 8 && sh_b == 0 && (x0 & 3) == 0 && x1 - x0 <= TILE) {
        int ja = x0 >> 2, jb = x1 >> 2;                           /* whole groups in the run ... */
        ja = ja < 1 ? 1 : ja;                                     /* ... with both neighbour cells */
        jb = jb > BW - 1 ? BW - 1 : jb;
        if (jb > ja) {
            tone_gen(orow, a, ax, bc, bx0, bw8, x0, 4 * ja, f, sh_r, sh_g, sh_b);
            tone_ab4(orow + 4 * ja, a + 3 * (4 * ja - ax), bc, ja, jb - ja);
            tone_gen(orow, a, ax, bc, bx0, bw8, 4 * jb, x1, f, sh_r, sh_g, sh_b);
            return;
        }
    }
    tone_gen(orow, a, ax, bc, bx0, bw8, x0, x1, f, sh_r, sh_g, sh_b);
}

/* one sprite into a fixed-point RGB buffer whose pixel (bx, by) is at buf[0] (rows `stride` values apart), over
 * the already-clipped rows [ya, yb] and columns [xa, xb]. q = (distance / radius)^2 is stepped along a row by
 * integer second differences (x 2^20), so a pixel costs integer adds, one table read and three multiplies -
 * no float, no transfer from the FPU. Saturating adds: a hot spot clips at 256, it does not wrap */
static inline void splat_one(const Spr *s, int xa, int xb, int ya, int yb, uint16_t *buf, int bx, int by, int stride) {
    const float QS = 1048576.0f;
    const float is = s->inv_r;
    const float dx0 = ((float)xa + 0.5f - s->sx) * is;          /* |dx0|, |dy| <= (R + 0.5) / r <= 3 */
    const int32_t qx0 = (int32_t)(dx0 * dx0 * QS);
    const int32_t d10 = (int32_t)((2.0f * dx0 * is + is * is) * QS);
    const int32_t d2 = (int32_t)(2.0f * is * is * QS);
    const uint32_t fr = s->fr, fg = s->fg, fb = s->fb;
    for (int y = ya; y <= yb; y++) {
        float dy = ((float)y + 0.5f - s->sy) * is;
        int32_t qy = (int32_t)(dy * dy * QS);
        if ((qy >> 10) >= SPR2_N) continue;                     /* 2^20 / SPR2_N = 2^10 */
        uint16_t *p = buf + (size_t)(y - by) * (size_t)stride + 3 * (size_t)(xa - bx);
        int32_t q = qx0 + qy, d1 = d10;
        for (int xx = xa; xx <= xb; xx++, p += 3) {
            int32_t qi = q >> 10;
            q += d1;
            d1 += d2;
            qi &= ~(qi >> 31);                                  /* rounding can dip below 0 at the centre */
            if (qi >= SPR2_N) continue;
            uint32_t w = g_prof16[qi], v;
            v = p[0] + ((fr * w) >> 16);
            p[0] = (uint16_t)(v < 65535u ? v : 65535u);
            v = p[1] + ((fg * w) >> 16);
            p[1] = (uint16_t)(v < 65535u ? v : 65535u);
            v = p[2] + ((fb * w) >> 16);
            p[2] = (uint16_t)(v < 65535u ? v : 65535u);
        }
    }
}

/* line yy of the bloom (quarter-resolution fixed-point U, lerped between its two nearest rows - the weights are
 * exact eighths, as across) for cells [c_lo, c_hi]; with the last cell, a copy past the edge (a zero-weight
 * bilinear read touches it). Integer: done in float per tile row, this cost ~19 cycles an output pixel */
static void bloom_row(uint16_t *bc, const uint16_t *U, int BW, int BH, int yy, int c_lo, int c_hi) {
    int y0 = ((yy + 2) >> 2) - 1;                             /* floor((yy + 0.5) / 4 - 0.5) */
    uint32_t w = (uint32_t)((2 * (yy & 3) + 5) & 7);           /* 5, 7, 1, 3 eighths for yy % 4 = 0..3 */
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

/* ---------------------------------------------------------------------------------------- small maths */
static inline float fract(float x) { return x - floorf(x); }

/* sin(x) to ~1e-3, no libm call (sinf costs the A53 ~100+ cycles; this is floorf = vrintm + a few
 * multiplies): the phase folded into [-pi, pi), a parabola, one correction. For the idle drift and the
 * twinkle - four a drone, every frame */
static inline float fsin(float x) {
    float t = x * 0.15915494f;
    t -= floorf(t + 0.5f);
    float y = 8.0f * t - 16.0f * t * fabsf(t);
    return 0.225f * (y * fabsf(y) - y) + y;
}

static inline float fcos(float x) { return fsin(x + 1.5707963f); }

static inline float c01(float v) { return fminf(fmaxf(v, 0.0f), 1.0f); }   /* NaN -> 0, no branches */

/* THREE.Color.setHSL, branch-free (called per drone per frame; every float compare that picks a branch stalls
 * the A53): q = l(1+s) below l = 0.5, l + s - ls above = l + s min(l, 1-l); three.js's hue2rgb ramp is
 * p + (q - p) clamp(min(6t, 4 - 6t), 0, 1) for t in [0, 1) */
static inline void hsl(float h, float s, float l, float *out) {
    h = fract(h);
    s = c01(s);
    l = c01(l);
    float q = l + s * fminf(l, 1.0f - l), p = 2.0f * l - q;
    for (int c = 0; c < 3; c++) {
        float t = fract(h + (1.0f - (float)c) * (1.0f / 3.0f));     /* r: h + 1/3, g: h, b: h - 1/3 */
        out[c] = p + (q - p) * c01(fminf(6.0f * t, 4.0f - 6.0f * t));
    }
}

static uint32_t hash3(int x, int y, int z) {                 /* visMath.ts hash3 */
    uint32_t h = (uint32_t)x * 374761393u + (uint32_t)y * 668265263u + (uint32_t)z * 1274126177u;
    h = (h ^ (h >> 13)) * 1103515245u;
    h ^= h >> 16;
    return h;
}

static float vnoise3(float x, float y, float z) {            /* visMath.ts valueNoise3, [-1, 1] */
    float fx0 = floorf(x), fy0 = floorf(y), fz0 = floorf(z);
    if (!(fabsf(fx0) < 1.0e6f && fabsf(fy0) < 1.0e6f && fabsf(fz0) < 1.0e6f)) return 0.0f;
    int xi = (int)fx0, yi = (int)fy0, zi = (int)fz0;
    float tx = x - fx0, ty = y - fy0, tz = z - fz0;
    float sx = tx * tx * (3.0f - 2.0f * tx), sy = ty * ty * (3.0f - 2.0f * ty), sz = tz * tz * (3.0f - 2.0f * tz);
    const float k = 1.0f / 4294967296.0f;
#define H(a, b, c) ((float)hash3(xi + (a), yi + (b), zi + (c)) * k)
    float v00 = H(0, 0, 0) + (H(1, 0, 0) - H(0, 0, 0)) * sx;
    float v10 = H(0, 1, 0) + (H(1, 1, 0) - H(0, 1, 0)) * sx;
    float v01 = H(0, 0, 1) + (H(1, 0, 1) - H(0, 0, 1)) * sx;
    float v11 = H(0, 1, 1) + (H(1, 1, 1) - H(0, 1, 1)) * sx;
#undef H
    float a = v00 + (v10 - v00) * sy, b = v01 + (v11 - v01) * sy;
    return (a + (b - a) * sz) * 2.0f - 1.0f;
}

/* the same noise over the face at lattice coordinate c of axis ax (0 x, 1 y, 2 z): bilinear over the other two
 * coordinates, whose cells + smoothed fractions are (ia, sa) and (ib, sb), in x, y, z order */
static float vn_face(int ax, int c, int ia, int ib, float sa, float sb) {
    const float k = 1.0f / 4294967296.0f;
    float h[4];
    for (int j = 0; j < 4; j++) {
        int a = ia + (j & 1), b = ib + (j >> 1);
        h[j] = (float)(ax == 0 ? hash3(c, a, b) : ax == 1 ? hash3(a, c, b) : hash3(a, b, c)) * k;
    }
    float l0 = h[0] + (h[1] - h[0]) * sa, l1 = h[2] + (h[3] - h[2]) * sa;
    return (l0 + (l1 - l0) * sb) * 2.0f - 1.0f;
}

/* vnoise3 at the point whose coordinate on axis ax is m and whose other two are pa, pb (x, y, z order), as
 * the two faces around m, e0 + (e1 - e0) smoothstep(fract(m)): trilinear interpolation in another order, so
 * equal to vnoise3 up to rounding. A nebula drone's anchor is fixed and time slides one coordinate of each of
 * its three noises, so its faces change only when that coordinate crosses a cell (every ~4.5 s): they are
 * cached per drone (slot = drone x 3 + noise), and a frame costs a lerp instead of eight hashes a noise */
static float neb_noise(int slot, int ax, float m, float pa, float pb) {
    float fm = floorf(m);
    if (!(fabsf(fm) < 1.0e6f)) return 0.0f;
    int c = (int)fm;
    float *e = &S.neb_e[slot * 2];
    if (c != S.neb_cell[slot]) {
        float fa = floorf(pa), fb = floorf(pb), ta = pa - fa, tb = pb - fb;
        float sa = ta * ta * (3.0f - 2.0f * ta), sb = tb * tb * (3.0f - 2.0f * tb);
        int ia = (int)fa, ib = (int)fb;
        e[0] = vn_face(ax, c, ia, ib, sa, sb);
        e[1] = vn_face(ax, c + 1, ia, ib, sa, sb);
        S.neb_cell[slot] = c;
    }
    float f = m - fm, s = f * f * (3.0f - 2.0f * f);
    return e[0] + (e[1] - e[0]) * s;
}

/* for the tests: the largest difference between the cached-face form and vnoise3 over n random points (their
 * slots are reset afterwards) */
float sw_noise_check(int n) {
    uint32_t r = 2463534242u;
    float worst = 0.0f;
    for (int i = 0; i < n; i++) {
        float p[3];
        for (int k = 0; k < 3; k++) {
            r ^= r << 13; r ^= r >> 17; r ^= r << 5;
            p[k] = ((float)(r >> 8) / 16777216.0f - 0.5f) * 200.0f;
        }
        int ax = i % 3;
        float pa = ax == 0 ? p[1] : p[0], pb = ax == 2 ? p[1] : p[2];
        S.neb_cell[0] = 0x7f7f7f7f;
        float d = fabsf(neb_noise(0, ax, p[ax], pa, pb) - vnoise3(p[0], p[1], p[2]));
        worst = d > worst ? d : worst;
    }
    S.neb_cell[0] = 0x7f7f7f7f;
    return worst;
}

static uint32_t g_rng = 12345u;
static float rnd(void) {                                      /* xorshift32, [0, 1) */
    g_rng ^= g_rng << 13;
    g_rng ^= g_rng >> 17;
    g_rng ^= g_rng << 5;
    return (float)(g_rng >> 8) * (1.0f / 16777216.0f);
}

/* free.vet's freqAt, by frequency: u in [0,1] -> the bin at a gentle log bias up to max_hz. The bins are
 * made once per spectrum size (a powf per drone per frame cost the A53 ~1 ms) */
static int bin_of(int nfft, float fft_hz, float u, float max_hz) {
    float top = max_hz / fft_hz;
    if (top > (float)(nfft - 1)) top = (float)(nfft - 1);
    float fb = 1.0f + floorf(powf(clampf(u, 0.0f, 1.0f), 1.6f) * (top - 1.0f));
    int b = fb < 1.0f ? 1 : (fb > (float)(nfft - 1) ? nfft - 1 : (int)fb);
    return b;
}

static void make_bins(int nfft, float fft_hz) {
    for (int i = 0; i < S.n; i++) S.bin_a[i] = (short)bin_of(nfft, fft_hz, (float)i / (float)S.n, 8400.0f);
    for (int c = 0; c < S.side; c++) S.bin_t[c] = (short)bin_of(nfft, fft_hz, (float)c / (float)(S.side - 1), 7200.0f);
    S.bins_hz = fft_hz;
    S.bins_nfft = nfft;
}

/* ---------------------------------------------------------------------------------------- cymatics: the plate */
/* free.vet's plate (source.ts cymaticsCloud "plate", drone.vert.glsl's standing wave): the drones are a grid across a
 * square plate that vibrates as a true standing wave - the antinodes pump up and down, the nodal lines hold still -
 * coloured by the mode shape (bright antinodes, dark nodal lines). (u, v) in [-0.5, 0.5]^2 x the plate's size in mode
 * lengths (free.vet's span: a bigger plate fits more nodes); the antisymmetric combination (mix -1) */
static inline float plate_field(float u, float v, float m, float n) {
    return fcos(m * PI_F * u) * fcos(n * PI_F * v) - fcos(n * PI_F * u) * fcos(m * PI_F * v);
}

/* source.ts cymaticsLadder(-1): m < n, sorted by the eigenfrequency m^2 + n^2 (ties: smaller m first) */
static const unsigned char LADDER[][2] = {
    {1, 2}, {1, 3}, {2, 3}, {1, 4}, {2, 4}, {3, 4}, {1, 5}, {2, 5}, {3, 5}, {1, 6}, {2, 6}, {4, 5},
    {3, 6}, {1, 7}, {4, 6}, {2, 7}, {3, 7}, {5, 6}, {1, 8}, {4, 7}, {2, 8}, {3, 8}, {5, 7}, {6, 7}};
#define LADDER_N ((int)(sizeof LADDER / sizeof LADDER[0]))
#define PLATE_L 9.0f                /* the plate's side at size 1, world units (it grows as size^0.25) */

/* ---------------------------------------------------------------------------------------- setup */
/* n drones (64..MAXN). 0, or a negative code. */
int sw_init(int n, int seed) {
    if (n < 64 || n > MAXN) return -1;
    memset(&S, 0, sizeof S);
    memset(S.neb_cell, 0x7f, sizeof S.neb_cell);                 /* 0x7f7f7f7f: no cell (they stay within 1e6) */
    g_rng = 12345u + (uint32_t)seed * 2654435761u;
    if (!g_rng) g_rng = 1u;
    S.n = n;
    for (int i = 0; i < n; i++) {
        S.seed[i] = rnd();
        float y = 1.0f - 2.0f * ((float)i + 0.5f) / (float)n;        /* fibDir */
        float r = sqrtf(fmaxf(0.0f, 1.0f - y * y));
        float a = (float)i * 2.399963229728653f;
        S.dirs[i * 3] = cosf(a) * r;
        S.dirs[i * 3 + 1] = y;
        S.dirs[i * 3 + 2] = sinf(a) * r;
        float rr = 3.1f * cbrtf(rnd()), th = rnd() * TAU, cph = 2.0f * rnd() - 1.0f;
        float sph = sqrtf(fmaxf(0.0f, 1.0f - cph * cph));
        S.anch[i * 3] = rr * sph * cosf(th);
        S.anch[i * 3 + 1] = rr * cph;
        S.anch[i * 3 + 2] = rr * sph * sinf(th);
    }
    int side = 2;
    while (side * side < n) side++;
    if (side > TER_MAX) side = TER_MAX;
    S.side = side;
    for (int i = 0; i < TUN_RINGS; i++) S.tun_hist[i] = 2.0f;
    S.pitch = 0.15f;
    S.camd = 12.0f;
    S.yaw_w = 1.0f;
    for (int i = 0; i < n; i++) S.vis[i] = S.tvis[i] = 1.0f;
    S.cym_m = (float)LADDER[0][0];
    S.cym_n = (float)LADDER[0][1];
    S.cym_last = -10.0f;
    if (!g_luts) luts_init();
    g_acc_dirty = 1;                                          /* clear whatever accumulator comes next, once */
    S.ready = 1;
    return 0;
}

/* ---------------------------------------------------------------------------------------- layout */
/* every drone's target (position, colour, light) for the style. v3's dials: amp (the ribbon's wave height), smooth
 * (0..1: the ribbon's wave / the orb's spikes smoothed away), pulse (the nebula's swell with the bass), frac (the
 * tunnel's share of lit drones), span (the plate's size in mode lengths) */
static void layout(int st, const float *fft, int nfft, float fft_hz, const float *wave, int nwave,
                   float bass, float level, float beat, float dt, float amp, float smooth, float pulse, float frac,
                   float span) {
    const int N = S.n;
    const float t = S.time;
    float c[3];
    (void)nfft;
    (void)fft_hz;
    for (int i = 0; i < N; i++) S.tvis[i] = 1.0f;                /* every drone lit, unless the figure says not */
    if (st == ST_CORONA) {
        for (int i = 0; i < N; i++) {
            float u = (float)i / N, mag = fft ? c01(fft[S.bin_a[i]]) : 0.0f;
            float th = u * TAU + t * 0.12f, r = 2.4f + S.seed[i] * 0.3f + mag * 3.2f + bass * 0.5f;
            S.tgt[i * 3] = fcos(th) * r;
            S.tgt[i * 3 + 1] = fsin(th) * r;
            S.tgt[i * 3 + 2] = (S.seed[i] - 0.5f) * (0.5f + mag * 1.4f);
            hsl(0.02f + u * 0.9f, 0.9f, 0.55f, c);
            memcpy(&S.tcol[i * 3], c, sizeof c);
        }
    } else if (st == ST_RIBBON) {
        /* the wave height: amp lifts it (and widens it, more gently: the ring never folds through its middle);
         * smooth calms it to a clean ring */
        const float keep = 1.0f - smooth, rk = sqrtf(amp) * keep, yk = amp * keep;
        for (int i = 0; i < N; i++) {
            float u = (float)i / N, s = 0.0f;
            if (wave && nwave > 1) s = clampf(wave[(int)(u * (float)(nwave - 1))], -1.0f, 1.0f);
            float ph = u * TAU + t * 0.06f, r = fmaxf(3.1f + s * 1.3f * rk, 0.4f);
            S.tgt[i * 3] = fcos(ph) * r;
            S.tgt[i * 3 + 1] = s * 1.5f * yk + fsin(ph * 2.0f + t * 0.4f) * 0.12f * keep;
            S.tgt[i * 3 + 2] = fsin(ph) * r;
            hsl(0.5f + u * 0.5f, 0.85f, 0.6f, c);
            memcpy(&S.tcol[i * 3], c, sizeof c);
        }
    } else if (st == ST_ORB) {
        /* a slow turn about the vertical axis (as v2), then the tumble about the horizontal one (P_TUMBLE) */
        float ca = cosf(t * 0.1f), sa = sinf(t * 0.1f), cb = cosf(S.tumble), sb = sinf(S.tumble);
        const float keep = 1.0f - smooth;
        for (int i = 0; i < N; i++) {
            float mag = fft ? c01(fft[S.bin_a[i]]) * keep : 0.0f;
            float r = 2.3f + mag * 2.1f + bass * 0.35f * keep;
            float dx = S.dirs[i * 3], dy = S.dirs[i * 3 + 1], dz = S.dirs[i * 3 + 2];
            float x0 = dx * ca + dz * sa, z0 = -dx * sa + dz * ca;
            S.tgt[i * 3] = x0 * r;
            S.tgt[i * 3 + 1] = (dy * cb - z0 * sb) * r;
            S.tgt[i * 3 + 2] = (dy * sb + z0 * cb) * r;
            hsl(0.62f - mag * 0.5f, 0.85f, 0.3f + mag * 0.45f, c);
            memcpy(&S.tcol[i * 3], c, sizeof c);
        }
    } else if (st == ST_NEBULA) {
        float tt = t * 0.22f, amp_n = 0.55f + pulse * bass * 2.4f + pulse * level * 0.6f, br = 1.0f + pulse * bass * 0.22f;
        const float NS = 0.5f;
        for (int i = 0; i < N; i++) {
            float bx = S.anch[i * 3], by = S.anch[i * 3 + 1], bz = S.anch[i * 3 + 2];
            float X = bx * NS, Y = by * NS, Z = bz * NS;
            /* = vnoise3(X + tt, Y, Z), vnoise3(X + 31.7, Y + tt, Z + 11.3), vnoise3(X + 71.1, Y + 47.9, Z + tt) */
            float n1 = neb_noise(i * 3, 0, X + tt, Y, Z);
            float n2 = neb_noise(i * 3 + 1, 1, Y + tt, X + 31.7f, Z + 11.3f);
            float n3 = neb_noise(i * 3 + 2, 2, Z + tt, X + 71.1f, Y + 47.9f);
            S.tgt[i * 3] = (bx + n1 * amp_n) * br;
            S.tgt[i * 3 + 1] = (by + n2 * amp_n) * br;
            S.tgt[i * 3 + 2] = (bz + n3 * amp_n) * br;
            hsl(0.72f + n1 * 0.14f, 0.8f, 0.42f + fabsf(n2) * 0.25f, c);
            memcpy(&S.tcol[i * 3], c, sizeof c);
        }
    } else if (st == ST_TUNNEL) {
        S.tun_hop -= dt;
        if (S.tun_hop <= 0.0f) {
            S.tun_hop = 0.07f;
            S.tun_ptr = (S.tun_ptr + 1) % TUN_RINGS;
            S.tun_hist[S.tun_ptr] = 2.0f + bass * 2.6f + beat * 0.5f;
        }
        S.tun_phase = fract(S.tun_phase + dt * (0.06f + level * 0.05f));
        /* a share of the drones lit, evenly: the rings and the drones on each both thin out with its square root, so
         * the spacing stays even both ways; the twist per unit length stays (the spiral keeps its pitch) */
        int rings = (int)((float)TUN_RINGS * sqrtf(frac) + 0.5f);
        rings = rings < 8 ? 8 : (rings > TUN_RINGS ? TUN_RINGS : rings);
        int per = (int)((float)N * frac / (float)rings + 0.5f);
        per = per < 3 ? 3 : per;
        if (per * rings > N) per = N / rings;
        const int lit = per * rings;
        const float twist = 0.31f * (float)TUN_RINGS / (float)rings;
        const float ZN = 8.0f, ZF = -26.0f;
        for (int i = 0; i < N; i++) {
            if (i >= lit) { S.tvis[i] = 0.0f; continue; }        /* not lit: fades out where it is */
            int ring = i / per, k = i % per;
            float zf = fract((float)ring / (float)rings + S.tun_phase);
            int row = ((S.tun_ptr - (int)(zf * TUN_RINGS)) % TUN_RINGS + TUN_RINGS) % TUN_RINGS;
            float ph = (float)k / per * TAU + ring * twist + t * 0.05f + S.roll;
            float r = S.tun_hist[row] + fsin(ph * 3.0f + t * 2.1f) * 0.07f;
            S.tgt[i * 3] = fcos(ph) * r;
            S.tgt[i * 3 + 1] = fsin(ph) * r;
            S.tgt[i * 3 + 2] = ZN + (ZF - ZN) * zf;
            hsl(fract(0.55f + zf * 0.5f), 0.85f, 0.55f - zf * 0.3f, c);
            memcpy(&S.tcol[i * 3], c, sizeof c);
        }
    } else if (st == ST_CYMATICS) {
        /* free.vet's plate: y = amp 0.5 field sin(2.4 t), amp = 0.7 (1 + 1.6 bass + 0.5 beat) at its plate side 5.2
         * (scaled to this plate's side); colour hsl(0.55 - 0.16 c, 0.85, 0.18 + 0.5 |c|), c = field / 2 */
        /* the plate grows a little with its size (the view keeps it framed) and its waves calm as its nodes crowd (a
         * bigger plate with free.vet's fixed amplitude reads as dust on a 64 x 64 grid) */
        const int side = S.side;
        const float sp = span, L = PLATE_L * powf(sp, 0.25f), m = S.cym_m, n = S.cym_n;
        const float wamp = 0.7f * (L / 5.2f) / sqrtf(sp) * (1.0f + bass * 1.6f + beat * 0.5f) * 0.5f * fsin(t * 2.4f);
        for (int i = 0; i < N; i++) {
            float u = (float)(i % side) / (float)(side - 1) - 0.5f;
            float v = (float)(i / side) / (float)(side - 1) - 0.5f;
            float f = plate_field(u * sp, v * sp, m, n), cc = f * 0.5f;
            S.tgt[i * 3] = u * L;
            S.tgt[i * 3 + 1] = wamp * f;
            S.tgt[i * 3 + 2] = v * L;
            hsl(0.55f - cc * 0.16f, 0.85f, 0.18f + fminf(fabsf(cc), 1.0f) * 0.5f, c);
            memcpy(&S.tcol[i * 3], c, sizeof c);
        }
    } else {                                                     /* terrain */
        const int side = S.side;
        S.hop_t -= dt;
        if (S.hop_t <= 0.0f) {
            S.hop_t = 0.05f;
            S.row_ptr = (S.row_ptr + 1) % side;
            for (int cI = 0; cI < side; cI++)
                S.hist[S.row_ptr * side + cI] = fft ? c01(fft[S.bin_t[cI]]) : 0.0f;
        }
        const float EX = 8.5f, EZ = 8.5f;
        for (int i = 0; i < N; i++) {
            int cI = i % side, rI = i / side;
            if (rI >= side) rI = side - 1;
            int row = ((S.row_ptr - rI) % side + side) % side;
            float h = S.hist[row * side + cI];
            S.tgt[i * 3] = ((float)cI / (side - 1) - 0.5f) * EX;
            S.tgt[i * 3 + 1] = -1.2f + h * 3.0f;
            S.tgt[i * 3 + 2] = (0.5f - (float)rI / (side - 1)) * EZ;
            hsl(0.62f - h * 0.45f, 0.9f, 0.25f + h * 0.55f, c);
            memcpy(&S.tcol[i * 3], c, sizeof c);
        }
    }
}

/* camera tilt per style (radians, looking down): the ribbon and terrain read from above, the plate from a
 * steep angle (its standing wave shows), the tunnel is flown level */
static const float PITCH[NSTYLE] = { 0.12f, 0.7f, 0.55f, 0.15f, 0.2f, 0.0f, 0.75f };
/* camera distance per style: the corona's low end blooms far out (free.vet: "the low end blooms the whole
 * disc"), so it is seen from further back than the rest */
static const float CAMD[NSTYLE] = { 15.0f, 12.0f, 12.0f, 13.0f, 12.0f, 12.0f, 13.0f };
/* where a turn (P_TURN) pivots, on the z axis: the tunnel (z 8 .. -26) turns about its middle, so a side view
 * shows all of it; the rest about their centre */
static const float PIVZ[NSTYLE] = { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, -9.0f, 0.0f };

static inline float smoother(float x) { return x * x * x * (x * (x * 6.0f - 15.0f) + 10.0f); }

/* ---------------------------------------------------------------------------------------- the final pass, on two cores */
/* The final pass (splat, bloom, tone map, trails, upscale) is split into bands - a row of 32 x 32 tiles each - that
 * write disjoint rows of the screen (and of the trails accumulator): the engine's thread and one worker take bands
 * in turn from a shared counter until none are left. Every band is computed exactly as the single-threaded loop did,
 * so the picture is the same pixel for pixel whichever thread draws which band. The worker is started on the first
 * frame that asks for two threads; it sleeps between frames */
typedef struct {
    uint8_t *dst;
    int dW, dH, dpitch, down, sh_r, sh_g, sh_b;
    int W, H, TX, TY, BW, BH, use_bloom, trails, prefilled;
    uint32_t bgpix, d16;
    const uint16_t *U;
    uint16_t *acc16;
    const int *bx0;
    const uint8_t *bw8;
} Job;

typedef struct {                   /* one thread's scratch */
    uint16_t L[TILE * TILE * 3];   /* a tile's light (6 KB: stays in L1) */
    uint16_t bcell[3 * (ROW_MAX / 4 + 2)];   /* one line of the bloom (+ a spare cell) */
    uint32_t row[ROW_MAX];         /* a line before the block upscale (render scale > 1) */
    uint32_t left[TILES_MAX];      /* trails: per tile of the band, the light left after decay */
} Scratch;

static Scratch g_scr[2];
static Job g_job;
static int g_next_band;            /* the next band to take (atomic) */
static int g_threads = 2;          /* sw_threads(): 1 = all on the engine's thread */
static int g_worker_state = 0;     /* 0 not started, 1 running, -1 could not start */
static pthread_t g_worker;
static pthread_mutex_t g_mx = PTHREAD_MUTEX_INITIALIZER;
static pthread_cond_t g_cv_go = PTHREAD_COND_INITIALIZER, g_cv_done = PTHREAD_COND_INITIALIZER;
static unsigned g_go_seq = 0, g_done_seq = 0;

/* without trails: a band tile by tile. A lit tile's light gathers in a 6 KB buffer that stays in L1 (its bin's
 * sprites, clipped to it) and goes to the screen from there. No screen-sized accumulator is touched at all
 * (streaming one through a 512 KB L2 that the compositor, on the next core, keeps flushing cost ~70 ns a lit pixel) */
static void band_tiles(const Job *J, int ty, Scratch *X) {
    const int TX = J->TX, W = J->W, H = J->H, BW = J->BW, BH = J->BH, down = J->down;
    const int y0 = ty * TILE, y1 = y0 + TILE < H ? y0 + TILE : H;
    uint16_t *L = X->L;
    for (int tx = 0; tx < TX; tx++) {
        const int x0 = tx * TILE, x1 = x0 + TILE < W ? x0 + TILE : W, t = ty * TX + tx;
        const int f = g_tflag[t] & (J->use_bloom ? 3 : 1);
        if (!f && J->prefilled)
            continue;                                     /* dark, and the screen already has it */
        if (f & 1) {
            memset(L, 0, sizeof(uint16_t) * 3 * TILE * (size_t)(y1 - y0));
            for (int e = g_tstart[t]; e < g_tstart[t + 1]; e++) {
                const Spr *s = &g_bin[e];
                const int xa = s->xa < x0 ? x0 : s->xa, xb = s->xb >= x1 ? x1 - 1 : s->xb;
                const int ya = s->ya < y0 ? y0 : s->ya, yb = s->yb >= y1 ? y1 - 1 : s->yb;
                if (xa > xb || ya > yb) continue;             /* (a binned box always reaches it) */
                splat_one(s, xa, xb, ya, yb, L, x0, y0, 3 * TILE);
            }
        }
        /* the bloom cells this tile reads: (x0 >> 2) - 1 .. ((x1 - 1) >> 2) + 1 */
        int c_lo = (x0 >> 2) - 1, c_hi = ((x1 - 1) >> 2) + 1;
        c_lo = c_lo < 0 ? 0 : c_lo;
        c_hi = c_hi > BW - 1 ? BW - 1 : c_hi;
        for (int yy = y0; yy < y1; yy++) {
            uint32_t *orow = down == 1 ? (uint32_t *)(J->dst + (size_t)yy * (size_t)J->dpitch) : X->row;
            if (!f) {
                for (int xx = x0; xx < x1; xx++) orow[xx] = J->bgpix;
            } else {
                if (f & 2)
                    bloom_row(X->bcell, J->U, BW, BH, yy, c_lo, c_hi);
                tone_fx(orow, L + (size_t)(yy - y0) * (3 * TILE), x0, X->bcell, J->bx0, J->bw8, BW, x0, x1, f,
                        J->sh_r, J->sh_g, J->sh_b);
            }
            if (down > 1)                                     /* block upscale into the screen */
                for (int r = 0; r < down; r++) {
                    int oy = yy * down + r;
                    if (oy >= J->dH) break;
                    uint32_t *o = (uint32_t *)(J->dst + (size_t)oy * (size_t)J->dpitch);
                    for (int xx = x0; xx < x1; xx++)
                        for (int k = 0; k < down; k++) {
                            int ox = xx * down + k;
                            if (ox < J->dW) o[ox] = X->row[xx];
                        }
                }
        }
    }
}

/* with trails: a band through the persistent accumulator (the caller's buffer, as uint16 light: W x H x 3 of them
 * fit in its first half), which holds the decaying light: the band's bins are splatted onto it (each copy clipped to
 * its tile), then line by line it is tone mapped and decayed - whole lines, which the prefetcher streams from memory
 * (tile by tile, a full screen of trails was 15 % slower). A tile whose light has all decayed to 0 is dark again
 * (g_tres): trails cost what they cover, not the whole screen */
static void band_trails(const Job *J, int ty, Scratch *X) {
    const int TX = J->TX, W = J->W, H = J->H, BW = J->BW, BH = J->BH, down = J->down;
    const int y0 = ty * TILE, y1 = y0 + TILE < H ? y0 + TILE : H;
    const unsigned char *tf = g_tflag + (size_t)ty * (size_t)TX;
    uint16_t *acc16 = J->acc16;
    uint32_t *left = X->left;
    int band_bloom = 0;
    for (int tx = 0; tx < TX; tx++) {
        const int x0 = tx * TILE, x1 = x0 + TILE < W ? x0 + TILE : W, t = ty * TX + tx;
        for (int e = g_tstart[t]; e < g_tstart[t + 1]; e++) {
            const Spr *s = &g_bin[e];
            const int xa = s->xa < x0 ? x0 : s->xa, xb = s->xb >= x1 ? x1 - 1 : s->xb;
            const int ya = s->ya < y0 ? y0 : s->ya, yb = s->yb >= y1 ? y1 - 1 : s->yb;
            if (xa > xb || ya > yb) continue;
            splat_one(s, xa, xb, ya, yb, acc16, 0, 0, 3 * W);
        }
        left[tx] = 0;
        band_bloom |= J->use_bloom ? tf[tx] & 2 : 0;
    }
    for (int yy = y0; yy < y1; yy++) {
        uint16_t *a = acc16 + 3 * (size_t)yy * (size_t)W;
        /* at render scale 1 the pixels go straight into the screen row; otherwise via the scratch row, then upscaled */
        uint32_t *orow = down == 1 ? (uint32_t *)(J->dst + (size_t)yy * (size_t)J->dpitch) : X->row;
        if (band_bloom)
            bloom_row(X->bcell, J->U, BW, BH, yy, 0, BW - 1);
        for (int tx = 0; tx < TX; tx++) {
            const int x0 = tx * TILE, x1 = x0 + TILE < W ? x0 + TILE : W;
            const int f = tf[tx] & (J->use_bloom ? 3 : 1);
            if (!f) {
                if (!J->prefilled)
                    for (int xx = x0; xx < x1; xx++) orow[xx] = J->bgpix;
                continue;
            }
            tone_fx(orow, a, 0, X->bcell, J->bx0, J->bw8, BW, x0, x1, f, J->sh_r, J->sh_g, J->sh_b);
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
            continue;                                         /* already in the screen */
        int oy0 = yy * down;
        if (oy0 >= J->dH) break;
        uint32_t *o = (uint32_t *)(J->dst + (size_t)oy0 * (size_t)J->dpitch);
        int ox = 0;
        for (int xx = 0; xx < W && ox < J->dW; xx++) {
            uint32_t v = X->row[xx];
            for (int k = 0; k < down && ox < J->dW; k++) o[ox++] = v;
        }
        for (int r = 1; r < down && oy0 + r < J->dH; r++)
            memcpy(J->dst + (size_t)(oy0 + r) * (size_t)J->dpitch, o, (size_t)J->dW * 4u);
    }
    for (int tx = 0; tx < TX; tx++)                           /* 0: this part of the accumulator is all 0 */
        g_tres[ty * TX + tx] = left[tx] != 0;
}

static void run_bands(const Job *J, Scratch *X) {
    for (;;) {
        int ty = __atomic_fetch_add(&g_next_band, 1, __ATOMIC_RELAXED);
        if (ty >= J->TY)
            break;
        if (J->trails) band_trails(J, ty, X);
        else band_tiles(J, ty, X);
    }
}

static void *worker_main(void *arg) {
    (void)arg;
    unsigned seen = 0;
    for (;;) {
        pthread_mutex_lock(&g_mx);
        while (g_go_seq == seen)
            pthread_cond_wait(&g_cv_go, &g_mx);
        seen = g_go_seq;
        pthread_mutex_unlock(&g_mx);
        run_bands(&g_job, &g_scr[1]);
        pthread_mutex_lock(&g_mx);
        g_done_seq = seen;
        pthread_cond_signal(&g_cv_done);
        pthread_mutex_unlock(&g_mx);
    }
    return NULL;
}

/* 1 or 2 threads for the final pass (2 by default); returns the number in use */
int sw_threads(int n) {
    g_threads = n >= 2 ? 2 : 1;
    return g_threads == 2 && g_worker_state >= 0 ? 2 : 1;
}

/* the bands of g_job: on both threads when the worker runs (the mutex hands the job over and the result back, so
 * each side sees the other's writes), else all here */
static void run_final(void) {
    __atomic_store_n(&g_next_band, 0, __ATOMIC_RELAXED);
    if (g_threads == 2 && g_worker_state == 0) {
        pthread_attr_t at;
        pthread_attr_init(&at);
        pthread_attr_setstacksize(&at, 256 * 1024);
        g_worker_state = pthread_create(&g_worker, &at, worker_main, NULL) == 0 ? 1 : -1;
        pthread_attr_destroy(&at);
    }
    if (g_threads == 2 && g_worker_state == 1 && g_job.TY > 1) {
        pthread_mutex_lock(&g_mx);
        unsigned seq = ++g_go_seq;
        pthread_cond_signal(&g_cv_go);
        pthread_mutex_unlock(&g_mx);
        run_bands(&g_job, &g_scr[0]);
        pthread_mutex_lock(&g_mx);
        while (g_done_seq != seq)
            pthread_cond_wait(&g_cv_done, &g_mx);
        pthread_mutex_unlock(&g_mx);
    } else {
        run_bands(&g_job, &g_scr[0]);
    }
}

/* ---------------------------------------------------------------------------------------- the frame */
/* dst: 32-bit pixels, dW x dH, dpitch bytes a row. acc: W*H*3 floats with W = ceil(dW/down), H = ceil(dH/down).
 * bloom: BW*BH*6 floats with BW = ceil(W/4), BH = ceil(H/4) (two planes). fft: nfft bins 0..1 (bin k = k *
 * fft_hz Hz), wave: nwave samples -1..1; either may be NULL (0 bins / samples). P: P_COUNT params.
 * Returns 0, or < 0 if the arguments were rejected (then nothing is written). */
int sw_frame(uint8_t *dst, int dW, int dH, int dpitch, int down, int sh_r, int sh_g, int sh_b,
             float *acc, int W, int H, float *bloom, int BW, int BH,
             const float *fft, int nfft, float fft_hz, const float *wave, int nwave, const float *P)
{
    if (!S.ready) return -9;
    if (!dst || !acc || !bloom || !P) return -1;
    if (dW <= 0 || dH <= 0 || dW > ROW_MAX || dH > ROW_MAX || dpitch < dW * 4) return -2;
    if (nfft < 0 || nfft > 65536 || nwave < 0 || nwave > 65536 || (nfft > 0 && !fft) || (nwave > 0 && !wave)) return -3;
    if (sh_r == sh_g || sh_r == sh_b || sh_g == sh_b) return -4;
    for (int k = 0; k < 3; k++) {
        int s = k == 0 ? sh_r : k == 1 ? sh_g : sh_b;
        if (s != 0 && s != 8 && s != 16 && s != 24) return -4;
    }
    if ((dpitch & 3) || ((uintptr_t)dst & 3)) return -5;
    if (down < 1 || down > 8 || W != (dW + down - 1) / down || H != (dH + down - 1) / down) return -6;
    if (BW != (W + 3) / 4 || BH != (H + 3) / 4) return -6;
    const int TX = (W + TILE - 1) / TILE, TY = (H + TILE - 1) / TILE, NT = TX * TY;
    if (NT > TILES_MAX) return -8;
    if (S.n > g_cap) {
        /* a box is <= 2 MAXR + 1 = 49 <= 2 TILE + 1 pixels wide: it touches <= 3 x 3 tiles (at 360 lines the size
         * cap keeps it to 2 x 2; the pages of the bins that are never written are never touched) */
        Spr *nb = (Spr *)realloc(g_spr_list, sizeof(Spr) * (size_t)S.n);
        if (!nb) return -7;
        g_spr_list = nb;
        Spr *nbin = (Spr *)realloc(g_bin, sizeof(Spr) * 9 * (size_t)S.n);
        if (!nbin) return -7;
        g_bin = nbin;
        g_cap = S.n;
    }
    if (nfft < 2) fft = NULL;
    if (nwave == 0) wave = NULL;
    if (!(fft_hz > 0.0f && fft_hz < 1.0e5f)) fft = NULL;
    if (fft && (S.bins_nfft != nfft || S.bins_hz != fft_hz)) make_bins(nfft, fft_hz);
    double ts0 = now_ms();

    const float dt = clampf(P[P_DT], 0.0f, 0.1f);
    const float level = clampf(P[P_LEVEL], 0.0f, 1.0f), bass = clampf(P[P_BASS], 0.0f, 1.0f);
    const float treble = clampf(P[P_TREBLE], 0.0f, 1.0f), beat = clampf(P[P_BEAT], 0.0f, 1.0f);
    const int beat_edge = P[P_BEAT_EDGE] > 0.5f, trig = P[P_TRIG] > 0.5f;
    S.time += dt;
    if (S.time > 1.0e5f) { S.time = 0.0f; S.cym_last = -10.0f; }  /* keeps the phases in float range */
    /* version 3's dials (see the enum; out of range or NaN: clamped, hold to 1) */
    const float pulse = clampf(P[P_PULSE], 0.0f, 4.0f), spin_amt = clampf(P[P_SPIN], 0.0f, 4.0f);
    const float burst_amp = clampf(P[P_BURST_AMP], 0.0f, 4.0f);
    const float hold = P[P_HOLD] == P[P_HOLD] ? clampf(P[P_HOLD], 0.1f, 4.0f) : 1.0f;
    const float smooth_amt = clampf(P[P_SMOOTH], 0.0f, 1.0f), wave_amp = clampf(P[P_AMP], 0.0f, 4.0f);
    const float frac = clampf(P[P_FRAC], 0.1f, 1.0f), span = clampf(P[P_SPAN], 0.25f, 4.0f);

    /* style: a knob value, or -1 = auto (the next style every 16 kicks, or 20 s without any) */
    float ps = P[P_STYLE];
    int want;
    if (ps < -0.5f) {
        if (!S.auto_mode) { S.auto_mode = 1; S.auto_beats = 0; S.auto_t = 0.0f; }
        S.auto_t += dt;
        if (beat_edge) S.auto_beats++;
        want = S.have_style ? S.style : 0;
        if (S.auto_beats >= 16 || S.auto_t >= 20.0f) {
            want = (want + 1) % NSTYLE;
            S.auto_beats = 0;
            S.auto_t = 0.0f;
        }
    } else {
        S.auto_mode = 0;
        want = (int)clampf(ps, 0.0f, (float)(NSTYLE - 1));
    }
    /* the plate: a kick steps it up its mode ladder - at most every 0.9 s, as free.vet's beat sweep (fast tracks melt
     * figure into figure rather than strobe) - and its mode numbers ease toward the step (non-integer plates are valid
     * Chladni fields: each figure flows into the next) */
    if (beat_edge && (want == ST_CYMATICS || S.style == ST_CYMATICS) && S.time - S.cym_last >= 0.9f) {
        S.cym_idx = (S.cym_idx + 1) % LADDER_N;
        S.cym_last = S.time;
    }
    {
        float mk = dt * 2.5f > 1.0f ? 1.0f : dt * 2.5f;
        S.cym_m += ((float)LADDER[S.cym_idx][0] - S.cym_m) * mk;
        S.cym_n += ((float)LADDER[S.cym_idx][1] - S.cym_n) * mk;
    }
    S.tumble += clampf(P[P_TUMBLE], -1.0f, 1.0f) * 0.6f * dt;    /* up to 0.6 rad/s */
    S.tumble -= TAU * floorf(S.tumble / TAU);
    int changed = !S.have_style || want != S.style;
    if (changed && S.have_style) {                                /* morph from where the drones are */
        memcpy(S.start, S.pos, sizeof(float) * 3 * (size_t)S.n);
        memcpy(S.scol, S.col, sizeof(float) * 3 * (size_t)S.n);
        S.morph = 0.0f;
        S.morphing = 1;
        S.trans_kind = S.trans_kind % 5 + 1;                        /* explode, implode, spiral, fade, fountain */
    }
    S.style = want;

    layout(S.style, fft, nfft, fft_hz, wave, nwave, bass, level, beat, dt, wave_amp, smooth_amt, pulse, frac, span);

    if (!S.have_style) {                                          /* first frame: snap into place */
        memcpy(S.pos, S.tgt, sizeof(float) * 3 * (size_t)S.n);
        memcpy(S.col, S.tcol, sizeof(float) * 3 * (size_t)S.n);
        memcpy(S.vis, S.tvis, sizeof(float) * (size_t)S.n);
        S.have_style = 1;
    }
    {                                                             /* lit drones fade in and out (the tunnel's share) */
        float kv = dt * 5.0f > 1.0f ? 1.0f : dt * 5.0f;
        for (int i = 0; i < S.n; i++) S.vis[i] += (S.tvis[i] - S.vis[i]) * kv;
    }
    const float stagger = 0.5f;
    if (S.morphing) {
        S.morph += dt / 1.6f;
        if (S.morph >= 1.0f) { S.morph = 1.0f; S.morphing = 0; }
        for (int i = 0; i < S.n; i++) {
            float local = clampf((S.morph - S.seed[i] * stagger) / (1.0f - stagger), 0.0f, 1.0f);
            float e = smoother(local);
            for (int k = 0; k < 3; k++) {
                S.pos[i * 3 + k] = S.start[i * 3 + k] + (S.tgt[i * 3 + k] - S.start[i * 3 + k]) * e;
                S.col[i * 3 + k] = S.scol[i * 3 + k] + (S.tcol[i * 3 + k] - S.scol[i * 3 + k]) * e;
            }
        }
    } else {
        float k = dt * 10.0f > 1.0f ? 1.0f : dt * 10.0f;        /* free.vet's fast ease */
        float kc = dt * 6.0f > 1.0f ? 1.0f : dt * 6.0f;
        for (int i = 0; i < S.n; i++) {
            float *p = &S.pos[i * 3];
            const float *g = &S.tgt[i * 3];
            float dz = g[2] - p[2];
            if (dz > 14.0f || dz < -14.0f) { p[0] = g[0]; p[1] = g[1]; p[2] = g[2]; }   /* a tunnel ring recycling */
            else { p[0] += (g[0] - p[0]) * k; p[1] += (g[1] - p[1]) * k; p[2] += dz * k; }
            for (int c = 0; c < 3; c++) S.col[i * 3 + c] += (S.tcol[i * 3 + c] - S.col[i * 3 + c]) * kc;
        }
    }

    /* camera: orbit (knob), the bass spin (free.vet's FX), per-style tilt; the tunnel is flown straight */
    S.spin += bass * bass * 1.2f * 0.5f * dt * spin_amt;
    float orbit = clampf(P[P_ORBIT], -1.0f, 1.0f);
    S.yaw += orbit * 0.6f * dt;
    if (S.yaw > 1.0e4f || S.yaw < -1.0e4f) S.yaw = 0.0f;
    if (S.spin > 1.0e4f) S.spin = 0.0f;
    float ck = dt * 2.0f > 1.0f ? 1.0f : dt * 2.0f;
    S.pitch += (PITCH[S.style] - S.pitch) * ck;
    S.camd += (CAMD[S.style] * clampf(P[P_CAMD_MUL], 0.1f, 10.0f) - S.camd) * ck;
    S.pivz += (PIVZ[S.style] - S.pivz) * ck;
    S.turn += (clampf(P[P_TURN], -PI_F, PI_F) - S.turn) * ck;
    S.roll += clampf(P[P_ROLL], -1.0f, 1.0f) * 1.2f * dt;          /* up to 1.2 rad/s */
    S.roll -= TAU * floorf(S.roll / TAU);                         /* [0, 2 pi): the rings' phase is periodic */
    /* the tunnel is flown straight down its bore: the orbit eases out (the short way round) and back */
    float ek = dt * 1.5f > 1.0f ? 1.0f : dt * 1.5f;
    S.yaw_w += ((S.style == ST_TUNNEL ? 0.0f : 1.0f) - S.yaw_w) * ek;
    float yw = S.yaw + S.spin;
    yw -= TAU * floorf(yw / TAU + 0.5f);                          /* (-pi, pi] */
    float yaw = yw * S.yaw_w;
    if (trig) S.burst = 1.0f;
    S.burst -= dt * 2.5f;
    if (S.burst < 0.0f) S.burst = 0.0f;

    /* ---- project + sort into tiles */
    double ts1 = now_ms();
    if (!g_luts) luts_init();
    const float cy = cosf(yaw), sy = sinf(yaw), cp = cosf(S.pitch), sp = sinf(S.pitch);
    const float camd = S.camd, halfW = 0.5f * (float)W, halfH = 0.5f * (float)H;
    const float fov = clampf(P[P_FOV], 0.2f, 2.5f);
    const float focal = halfH / tanf(0.5f * fov);
    const float res = (float)H / 360.0f;
    const float size = clampf(P[P_SIZE], 0.1f, 40.0f);
    const float cap = clampf(P[P_CAP], 1.0f, 16.0f);
    /* (written so that the neutral dials compute exactly v2's floats, in v2's order: swarm-cmp proves it) */
    const float audio_size = 1.0f + pulse * beat * 0.55f + pulse * level * 0.22f;   /* pulse */
    const float swell = (1.0f + pulse * bass * 0.12f + pulse * beat * 0.05f) * hold;  /* x the Trigger-held breath */
    const float bright = clampf(P[P_BRIGHT], 0.0f, 20.0f) * (1.0f + level * 0.6f + pulse * beat * 0.5f);
    const float shimmer = treble;
    const float hue = clampf(P[P_HUE], 0.0f, 1.0f) * TAU;          /* hue rotation about the grey axis */
    const float hc = cosf(hue), hs = sinf(hue), hk = 0.57735026919f;
    const int tmid = S.morphing ? S.trans_kind : 0;
    const float fxs = g_prof_max * LFX;                           /* splat: light x 256 at the peak */
    const float glow = clampf(P[P_GLOW], 0.0f, 1.0f);
    const float bloom_amt = glow < 0.5f ? glow * 2.0f * 1.4f : 1.4f;
    const int use_bloom = bloom_amt > 0.0f;
    float *b0 = bloom, *b1 = bloom + 3 * (size_t)BW * (size_t)BH;
    if (use_bloom) memset(b0, 0, sizeof(float) * 3 * (size_t)BW * (size_t)BH);
    memset(g_tstart, 0, sizeof(int) * (size_t)(NT + 1));
    /* the loop's invariants as locals: its float stores (sprites, bloom) might alias S as far as the compiler
     * knows, so it reloaded these from S for every drone (and re-tested the burst with a float compare) */
    const int n = S.n;
    const float stime = S.time, burst = S.burst, morph = S.morph;
    const int bursting = burst > 0.0f;
    const int turning = S.turn != 0.0f;                          /* exactly 0 unless a mode asks for a turn */
    const float ct = cosf(S.turn), st = sinf(S.turn), pivz = S.pivz;
    const float ph_x = stime * 0.6f, ph_y = stime * 0.5f, ph_z = stime * 0.7f;
    const float ph_tw = stime * (3.0f + shimmer * 10.0f);
    int ns = 0;
    for (int i = 0; i < n; i++) {
        const float vz = S.vis[i];
        if (vz < 0.01f) continue;                                  /* not lit in this figure */
        float x = S.pos[i * 3], y = S.pos[i * 3 + 1], z = S.pos[i * 3 + 2];
        float fade = vz;
        if (tmid) {                                                /* drone.vert.glsl's transition presets */
            float local = clampf((morph - S.seed[i] * stagger) / (1.0f - stagger), 0.0f, 1.0f);
            float mid = sinf(local * PI_F);
            if (tmid == 1) {
                float l = sqrtf(x * x + y * y + z * z) + 1e-4f;
                float a = mid * 2.5f / l;
                x += x * a; y += y * a; z += z * a;
            } else if (tmid == 2) {
                float s = 1.0f - 0.9f * mid;
                x *= s; y *= s; z *= s;
            } else if (tmid == 3) {
                float a = mid * 2.5f, ca = fcos(a), sa = fsin(a);
                float nx = ca * x + sa * z, nz = -sa * x + ca * z;
                x = nx; z = nz;
            } else if (tmid == 4) {
                fade = vz * (1.0f - 0.92f * mid);
            } else {
                y += mid * 3.0f;
            }
        }
        if (bursting) {                                            /* Trigger: burst outward, reform */
            float l = sqrtf(x * x + y * y + z * z) + 1e-4f;
            float a = burst * burst * 1.8f * burst_amp / l;
            x += x * a; y += y * a; z += z * a;
        }
        float sd = S.seed[i];                                      /* idle drift */
        x += fsin(ph_x + sd * 30.0f) * 0.025f;
        y += fcos(ph_y + sd * 22.0f) * 0.025f;
        z += fsin(ph_z + sd * 17.0f) * 0.025f;
        x *= swell; y *= swell; z *= swell;
        if (turning) {                                             /* the turn, about the style's pivot */
            float zr = z - pivz, xt = ct * x - st * zr;
            z = st * x + ct * zr + pivz;
            x = xt;
        }
        float x1 = cy * x - sy * z, z1 = sy * x + cy * z;
        float y2 = cp * y - sp * z1, z2 = sp * y + cp * z1;
        float d = camd - z2;
        if (!(d > 0.3f)) continue;                                /* behind or at the camera */
        /* branch-free from here to the bounds test (fminf/fmaxf are single instructions; a float compare that
         * picks a branch stalls the A53): screen position clamped far outside, then integer culling */
        float inv_d = 1.0f / d;
        float sxp = fminf(fmaxf(halfW + focal * x1 * inv_d, -1.0e6f), 1.0e6f);
        float syp = fminf(fmaxf(halfH - focal * y2 * inv_d, -1.0e6f), 1.0e6f);
        /* free.vet: gl_PointSize = clamp(size * pulse * 8 / depth, 1, 48) px at browser resolution; 48 px of a
         * ~1080-line canvas is 16 at 360 lines, and so is the cap here by default (near drones were 3x free.vet's
         * size, and a 49 x 49 sprite costs as much as a hundred small ones) */
        float diam = fminf(fmaxf(size * audio_size * 8.0f * inv_d, 1.0f), cap) * res;
        float r = fmaxf(0.5f * diam, 0.75f);
        int R = (int)ceilf(r);
        R = R > MAXR ? MAXR : R;
        int cx = (int)floorf(sxp), cyy = (int)floorf(syp);
        if (cx + R < 0 || cyy + R < 0 || cx - R >= W || cyy - R >= H) continue;
        float tw = 0.82f + (0.18f + 0.55f * shimmer) * fsin(ph_tw + sd * 120.0f);
        float cr = S.col[i * 3], cg = S.col[i * 3 + 1], cb = S.col[i * 3 + 2];
        if (hue != 0.0f) {                                         /* Rodrigues about (1,1,1)/sqrt(3) */
            float dot = hk * (cr + cg + cb);
            float kxr = hk * (cb - cg), kxg = hk * (cr - cb), kxb = hk * (cg - cr);
            float nr = cr * hc + kxr * hs + hk * dot * (1.0f - hc);
            float ng = cg * hc + kxg * hs + hk * dot * (1.0f - hc);
            float nb = cb * hc + kxb * hs + hk * dot * (1.0f - hc);
            cr = nr; cg = ng; cb = nb;
        }
        float amp = fminf(fmaxf(bright * tw * fade, 0.0f), 1.0e4f);   /* as the shader: per pixel, not area */
        const float ar = amp * fminf(fmaxf(cr, 0.0f), 4.0f), ag = amp * fminf(fmaxf(cg, 0.0f), 4.0f),
                    ab = amp * fminf(fmaxf(cb, 0.0f), 4.0f);
        const int xa = cx - R < 0 ? 0 : cx - R, xb = cx + R >= W ? W - 1 : cx + R;
        const int ya = cyy - R < 0 ? 0 : cyy - R, yb = cyy + R >= H ? H - 1 : cyy + R;
        Spr *s = &g_spr_list[ns++];
        s->sx = sxp;
        s->sy = syp;
        s->inv_r = 1.0f / r;
        s->fr = (uint16_t)fminf(ar * fxs, 65535.0f);              /* >= 0: amp and colour are */
        s->fg = (uint16_t)fminf(ag * fxs, 65535.0f);
        s->fb = (uint16_t)fminf(ab * fxs, 65535.0f);
        s->xa = (int16_t)xa;                                       /* 0 <= xa <= xb < W <= ROW_MAX: fit */
        s->xb = (int16_t)xb;
        s->ya = (int16_t)ya;
        s->yb = (int16_t)yb;
        {                                                          /* the tiles its box touches */
            const int tx0 = xa / TILE, tx1 = xb / TILE, ty0 = ya / TILE, ty1 = yb / TILE;
            int *c = &g_tstart[ty0 * TX + tx0 + 1];
            if (tx1 - tx0 <= 1 && ty1 - ty0 <= 1) {                /* <= 2 x 2 (always, at 360 lines): no loop */
                c[0]++;
                if (tx1 > tx0) c[1]++;
                if (ty1 > ty0) {
                    c[TX]++;
                    if (tx1 > tx0) c[TX + 1]++;
                }
            } else {
                for (int ty = ty0; ty <= ty1; ty++)
                    for (int tx = tx0; tx <= tx1; tx++)
                        g_tstart[ty * TX + tx + 1]++;
            }
        }
        if (use_bloom) {
            /* the bloom's source, straight from the sprites: each one's light, spread bilinearly over the
             * quarter-resolution cells around it, per full-resolution pixel (a cell is 4x4 of them). Same
             * as box-filtering the full accumulator down, which cost 8 ms a frame on the EYESY */
            float e = g_spr_energy * r * r * (1.0f / 16.0f);
            float fx = sxp * 0.25f - 0.5f, fy = syp * 0.25f - 0.5f;
            int gx = (int)floorf(fx), gy = (int)floorf(fy);
            float wx = fx - (float)gx, wy = fy - (float)gy;
            for (int k = 0; k < 4; k++) {
                int cx2 = gx + (k & 1), cy2 = gy + (k >> 1);
                if (cx2 < 0 || cy2 < 0 || cx2 >= BW || cy2 >= BH) continue;
                float w = ((k & 1) ? wx : 1.0f - wx) * ((k >> 1) ? wy : 1.0f - wy) * e;
                float *o = b0 + 3 * ((size_t)cy2 * (size_t)BW + (size_t)cx2);
                o[0] += ar * w;
                o[1] += ag * w;
                o[2] += ab * w;
            }
        }
    }
    /* binning (a counting sort): a copy of each sprite in the bin of every tile its box touches, so a tile reads
     * exactly the sprites that reach it, in one run (it used to scan its 3 x 3 neighbourhood's sprites through an
     * index - scattered reads, most of them clipped away) */
    for (int t = 0; t < NT; t++) g_tstart[t + 1] += g_tstart[t];
    memcpy(g_tfill, g_tstart, sizeof(int) * (size_t)NT);
    for (int i = 0; i < ns; i++) {
        const Spr *s = &g_spr_list[i];
        const int tx0 = s->xa / TILE, tx1 = s->xb / TILE, ty0 = s->ya / TILE, ty1 = s->yb / TILE;
        int *c = &g_tfill[ty0 * TX + tx0];
        if (tx1 - tx0 <= 1 && ty1 - ty0 <= 1) {                    /* <= 2 x 2, as counted */
            g_bin[c[0]++] = *s;
            if (tx1 > tx0) g_bin[c[1]++] = *s;
            if (ty1 > ty0) {
                g_bin[c[TX]++] = *s;
                if (tx1 > tx0) g_bin[c[TX + 1]++] = *s;
            }
        } else {
            for (int ty = ty0; ty <= ty1; ty++)
                for (int tx = tx0; tx <= tx1; tx++)
                    g_bin[g_tfill[ty * TX + tx]++] = *s;
        }
    }

    /* ---- trails (knob 3's upper half): a screen-sized accumulator keeps the decayed light of the frames
     * before. Without them the final pass gathers each lit tile's light in L1 and no screen-sized buffer is
     * touched: the accumulator is 3x the CM3's L2, and streaming it every frame costs more than everything
     * else together */
    double ts2 = now_ms();
    /* knob 3: bloom up to the middle, a dead band, then trails from 0.6 (starting at a visible 0.25: a trail
     * too faint to see would still cost the whole-frame path; the panel's knob rests near 0.51) */
    const float decay = glow >= 0.6f ? 0.25f + (glow - 0.6f) * (0.63f / 0.4f) : 0.0f;
    const int trails = decay > 0.0f;
    /* starting trails (or on a new buffer or size), the accumulator is cleared once, so no light from an earlier
     * trail session flashes back; then a tile's part of it is all 0 whenever g_tres says so */
    if (trails && (g_acc_dirty || acc != g_acc_last || W != g_acc_w || H != g_acc_h)) {
        memset(acc, 0, sizeof(float) * 3 * (size_t)W * (size_t)H);
        memset(g_tres, 0, sizeof g_tres);
    }
    g_acc_last = acc;
    g_acc_w = W;
    g_acc_h = H;
    g_acc_dirty = !trails;
    for (int t = 0; t < NT; t++)                  /* 1 = light: sprites reach it, or (trails) some is left there */
        g_tflag[t] = (unsigned char)(g_tstart[t + 1] > g_tstart[t] || (trails && g_tres[t]));

    /* ---- bloom: the sprites' light at quarter resolution (made during projection), its bright part,
     * blurred twice (separable box, radius 2) */
    double ts3 = now_ms();
    /* in fixed point from here: two uint16 planes in the (now free) second half of the bloom buffer */
    uint16_t *U = (uint16_t *)b1, *V = U + 3 * (size_t)BW * (size_t)BH;
    if (use_bloom) {
        const float thr = 0.35f, s = bloom_amt * LFX;
        const size_t nb = 3 * (size_t)BW * (size_t)BH;
        for (size_t k = 0; k < nb; k++)                           /* the bright part, as light x 256 */
            U[k] = (uint16_t)fminf(fmaxf(b0[k] - thr, 0.0f) * s, 65535.0f);
        /* two box blurs of radius 2, each separable, in integer (sums of 5 <= 327675, / 5 is a multiply).
         * Exact, so empty cells stay exactly 0 and light up no tile */
        const int RW = 3 * BW;
        for (int pass = 0; pass < 2; pass++) {
            for (int by = 0; by < BH; by++) {                      /* horizontal: U -> V */
                const uint16_t *restrict si = U + (size_t)by * (size_t)RW;
                uint16_t *restrict o = V + (size_t)by * (size_t)RW;
                for (int k = 6; k < RW - 6; k++)                    /* interior: all five taps inside */
                    o[k] = (uint16_t)(((uint32_t)si[k - 6] + si[k - 3] + si[k] + si[k + 3] + si[k + 6]) / 5u);
                for (int bx = 0; bx < BW; bx++) {                   /* the two cells at each edge: clamped */
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
            for (int by = 0; by < BH; by++) {                      /* vertical: V -> U */
                const uint16_t *r[5];
                for (int j = -2; j <= 2; j++) {
                    int y = by + j < 0 ? 0 : (by + j >= BH ? BH - 1 : by + j);
                    r[j + 2] = V + (size_t)y * (size_t)RW;
                }
                uint16_t *restrict o = U + (size_t)by * (size_t)RW;
                for (int k = 0; k < RW; k++)
                    o[k] = (uint16_t)(((uint32_t)r[0][k] + r[1][k] + r[2][k] + r[3][k] + r[4][k]) / 5u);
            }
        }
        /* the tiles the bloom reaches: a lit cell feeds full-resolution pixels [4 bx - 4, 4 bx + 7] (its own 4
         * plus the bilinear neighbours) */
        for (int by = 0; by < BH; by++) {
            int ya = by * 4 - 4, yb = by * 4 + 7;
            ya = ya < 0 ? 0 : ya;
            yb = yb >= H ? H - 1 : yb;
            for (int bx = 0; bx < BW; bx++) {
                const uint16_t *u = U + 3 * ((size_t)by * (size_t)BW + (size_t)bx);
                if (!(u[0] | u[1] | u[2])) continue;
                int xa = bx * 4 - 4, xb = bx * 4 + 7;
                xa = xa < 0 ? 0 : xa;
                xb = xb >= W ? W - 1 : xb;
                for (int ty = ya / TILE; ty <= yb / TILE; ty++)
                    for (int tx = xa / TILE; tx <= xb / TILE; tx++)
                        g_tflag[ty * TX + tx] |= 2;
            }
        }
    }

    /* ---- the final pass: + bloom, the tone curve over the background, trails, block upscale. Fixed point */
    double ts4 = now_ms();
    const float bgv[3] = { clampf(P[P_BG_R], 0.0f, 1.0f), clampf(P[P_BG_G], 0.0f, 1.0f), clampf(P[P_BG_B], 0.0f, 1.0f) };
    tm8_update(clampf(P[P_EXPOSURE], 0.0f, 100.0f), bgv);
    static int bx0[ROW_MAX];                                      /* bilinear bloom: columns once per frame */
    static uint8_t bw8[ROW_MAX];
    if (use_bloom)
        for (int xx = 0; xx < W; xx++) {
            float fx = ((float)xx + 0.5f) * 0.25f - 0.5f;
            int x0 = (int)floorf(fx);
            float w = fx - (float)x0;
            if (x0 < 0) { x0 = 0; w = 0.0f; }
            if (x0 >= BW - 1) { x0 = BW - 1; w = 0.0f; }
            bx0[xx] = x0;
            bw8[xx] = (uint8_t)(w * 8.0f + 0.5f);                   /* 5, 7, 1, 3 eighths (0 at the edges) */
        }
    const uint32_t bgpix = ((uint32_t)g_tm8[0][0] << sh_r) | ((uint32_t)g_tm8[1][0] << sh_g) |
                           ((uint32_t)g_tm8[2][0] << sh_b);
    /* the engine already filled the destination with exactly this colour (Persist off, same knob-5 colour as
     * last frame): the dark tiles need no writes at all (at render scale 1; ~2 ms of memory writes) */
    const int prefilled = P[P_PREFILLED] > 0.5f && down == 1;
    for (int t = 0; t < NT; t++) {
        int f = g_tflag[t] & (use_bloom ? 3 : 1);
        g_tiles[(f & 1) ? 0 : (f ? 1 : 2)] += 1.0;
    }
    Job *J = &g_job;
    J->dst = dst; J->dW = dW; J->dH = dH; J->dpitch = dpitch; J->down = down;
    J->sh_r = sh_r; J->sh_g = sh_g; J->sh_b = sh_b;
    J->W = W; J->H = H; J->TX = TX; J->TY = TY; J->BW = BW; J->BH = BH;
    J->use_bloom = use_bloom; J->trails = trails; J->prefilled = prefilled;
    J->bgpix = bgpix;
    J->d16 = (uint32_t)(decay * 65536.0f);                        /* <= 0.88: the products fit 32 bits */
    J->U = U;
    J->acc16 = (uint16_t *)acc;
    J->bx0 = bx0;
    J->bw8 = bw8;
    run_final();
    double ts5 = now_ms();
    g_stage[0] += ts1 - ts0;
    g_stage[1] += ts2 - ts1;
    g_stage[2] += ts3 - ts2;                                       /* the trails' setup (the splats are tiled) */
    g_stage[3] += ts4 - ts3;
    g_stage[4] += ts5 - ts4;
    g_stage[5] += 1.0;
    return 0;
}

/* for tests and DEBUG: 0 style, 1 morph, 2 morphing, 3 transition, 4 cymatics figure, 5 its eased m,
 * 6 pitch, 7 auto; 8..12 mean ms per frame since the last call: layout+motion, project, clear+splat,
 * bloom, final pass; 13..15 mean tiles per frame: lit, bloom only, dark (the totals reset when read with
 * n >= 13) */
int sw_debug(float *out, int n) {
    double fr = g_stage[5] > 0.0 ? g_stage[5] : 1.0;
    float v[16] = { (float)S.style, S.morph, (float)S.morphing, (float)S.trans_kind, (float)S.cym_idx,
                    S.cym_m, S.pitch, (float)S.auto_mode,
                    (float)(g_stage[0] / fr), (float)(g_stage[1] / fr), (float)(g_stage[2] / fr),
                    (float)(g_stage[3] / fr), (float)(g_stage[4] / fr),
                    (float)(g_tiles[0] / fr), (float)(g_tiles[1] / fr), (float)(g_tiles[2] / fr) };
    if (!out || n <= 0) return 16;
    for (int i = 0; i < n && i < 16; i++) out[i] = v[i];
    if (n >= 13) {
        memset(g_stage, 0, sizeof g_stage);
        memset(g_tiles, 0, sizeof g_tiles);
    }
    return 16;
}
