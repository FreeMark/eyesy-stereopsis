/* textswarm.c - the kernel of the EYESY mode "14 - Swarm - Text": free.vet's swarm sim writing a word in drones -
 * FREE.VET unless the mode is told otherwise, and a second line under it when asked - with the sim's neighbor physics
 * and its audio-FX rack, in the Swarm Visualizer's light (glow.h). "15 - Swarm - Flower of Life" runs a copy of it
 * (flowerswarm.c, written by tools/make_glow_modes.py: its own build, its own swarm) on the sim's Flower of Life
 * instead of a text (tx_shape: formations.ts flowerOfLife, its arithmetic).
 *
 * The text: the mode renders each line with pygame in its font at 220 px with 60 px of padding (the sim's canvas) and
 * hands the pixels to tx_line(), which turns them into slots exactly as the sim's sources.ts does (sampleText /
 * cloudFromImageData / collectOutline): a pixel is inside if its alpha > 60; the boundary pixels are chained nearest
 * first (within 3 px); every chain gets its share of the line's drones by length and is resampled evenly by arc
 * length. One line is the sim's: 4.5 units tall (at most 16 wide), centred, flat, coloured with its rainbow by x,
 * hsl(u 0.8 + 0.02, 0.85, 0.58). Two lines (tx_layout): both at the same scale (letters the same size), the first above
 * the second, the pair centred, the rainbow across the wider line. The second line has drones of its own: by default
 * as many as keep its dots as close together as the first line's. A changed line: its drones fly to the new text (the
 * sim's untangle: rank by azimuth, then height). A second line pours out of the first (each new drone starts at a drone
 * of the first line, dark, and fades in on its way) and leaves by fading out where it is.
 *
 * The physics: engine/swarm.ts stepSwarm on the live sim's path (no acceleration, jerk or downwash limits): every
 * drone springs toward its slot (x the swarm's scale) and pushes off any neighbour inside the mean of the two drones'
 * safe radii, f = (thresh - d) / thresh x avoidance; velocity damped by 1 - damping dt, clamped to the speed limit;
 * drones updated in place in order (as the sim); at most 48 neighbours visited. Two versions of it:
 *   ts_step       the sim's, line by line: arithmetic in double, state in float (its JavaScript numbers and
 *                 Float32Arrays), its spatial hash (1 << 14 slots, cells the size of the largest radius, the 27 cells
 *                 around a drone). The tests run it against the sim's own code: the same numbers (test_textswarm.py).
 *   ts_step_fast  what the mode runs: the same steps, the same hash and the same visits in the same order, in float,
 *                 sin and exp by polynomials (to ~1e-5); the hash kept as each slot's drones side by side with their
 *                 positions and radii (no list to chase), a bit per slot for "anything here" (most of the 27 slots
 *                 are empty: that answer stays in the cache), a cell's occupied slots reused by the drones after it in
 *                 the same cell; on a core of its own, a frame ahead of the picture (THREADS 2, the default: see "the
 *                 physics' own core"). It keeps the sim's walk on purpose: the text is flat, so 18 of the 27 cells only
 *                 ever find drones of other cells sharing their slot, which fail the distance test but count against
 *                 the 48 - in a crowd that leaves each drone fewer real neighbours, and Free's settings were made on
 *                 that look (a walk of the 9 real cells alone packs the 0.5x blob ~20 % looser).
 *                 test_textswarm.py fast holds it to ts_step.
 * Units: the mode passes light units (lu); 1 lu = 0.08 scene units (the sim's LIGHT_UNIT). z stays 0, not stored.
 *
 * The audio-FX rack (the sim's audioFx.ts, NeighborField.tsx, player.vert.glsl):
 *   Pulse     the dots swell 0.55 x beat + 0.22 x level and glow 0.6 x level + 0.5 x beat (x the amount)
 *   Push-away a share of the swarm (rank quantile < frac) inflates its safe radius by 2.2 x level x amount
 *   Waves     two travelling bass plane waves through the safe radii (knob 2 sets the amount)
 *   Ripples   every kick (the beat's rising edge) sends an expanding shell out of a random drone
 *   Bass swell the target space breathes: x (1 + 0.1 x bass x amount)
 *   Shimmer   treble deepens and speeds the twinkle
 *   Hue drift the palette turns about the grey axis with the mids (Rodrigues)
 * The wave field only changes safe radii (clamped x0.35 .. x4): the physics makes the motion, the swarm adapts.
 * Not here: the sim's bass spin (Free: no rotation or tilt) and its Flow (a curve thing).
 *
 * The view: the sim's "Constrain view" (camera straight on, dollying as the swarm's extent changes: out quickly, in
 * lazily), fitted to the swarm's box - its full width, as Free asked - instead of its bounding sphere.
 * The dots: the sim's sprite (glow.h's profile), light = brightness x twinkle x colour x 1.5 at the centre (as the
 * Swarm figures, calibrated against the sim); their size follows the zoom as perspective does, never under a pixel.
 * A faint field of stars behind, as the sim's.
 * Knobs: 1 the size (the target space, 0.5x .. 3x: small = crowded, the drones fight for room), 2 the waves, 3 glow
 * and trails (the Swarm's), 4 the colour (the gradient turned around the wheel), 5 the background. Trigger: a ripple
 * out of the middle of the text.
 */
#include "glow.h"

#define TS_VERSION 2
#define TS_MAXN 8192
#define TS_TABLE (1 << 14)         /* the sim's spatial hash */
#define TS_MASK (TS_TABLE - 1)
#define TS_MAX_NEIGH 48
#define TS_MAX_RIPPLES 4
#define TS_RIPPLE_LIFE 1.8
#define TS_RIPPLE_SPEED 7.0
#define TS_RIPPLE_W 0.9
#define TS_LU 0.08                 /* scene units per light unit (the sim's LIGHT_UNIT) */
#define TS_MAX_STARS 600
#define TS_TRIG_GAIN 3.2           /* a Trigger's ripple: twice a kick's at the sim's Ripples 1.0 */
#define TS_FADE 0.6                /* s: a drone that appears fades in over this, one that leaves fades out */

enum {
    P_DT, P_K1, P_K2, P_K3, P_K4, P_K5,
    P_FG_R, P_FG_G, P_FG_B, P_BG_R, P_BG_G, P_BG_B,
    P_LEVEL, P_BASS, P_MID, P_TREBLE, P_BEAT, P_KICK, P_TRIG, P_PREFILLED,
    P_WAVE_HZ,
    /* the sim's settings (the mode file's settings block) */
    P_SAFE,                        /* safe zone, lu (1.0) */
    P_SPEED,                       /* speed limit, lu/s (25) */
    P_RIGID,                       /* seek spring (6) */
    P_AVOID,                       /* separation strength (53) */
    P_DAMP,                        /* damping (3.5) */
    P_SIZE_MIN, P_SIZE_MAX,        /* knob 1's range (0.5 .. 3.0) */
    P_WAVES_MAX,                   /* knob 2's top (3.0) */
    P_PULSE, P_PUSH, P_PUSH_FRAC, P_RIPPLES, P_SWELL, P_SHIMMER, P_HUE_DRIFT,   /* the FX amounts; 0 = off */
    P_INTENSITY,                   /* the analysis gain (1.0) */
    P_BRIGHT,                      /* the sim's brightness (1.5) */
    P_DOT,                         /* a dot's radius, lu */
    P_DOT_MIN,                     /* ... never under this, px at 720 lines */
    P_STARS,                       /* how many stars (0 = none) */
    P_COUNT
};

typedef struct { double x, y, age, gain; } Ripple;

static struct {
    int ready, n, have_text;
    int n1;                        /* the first line's drones (the mode's COUNT) */
    float pos[TS_MAXN * 2], vel[TS_MAXN * 2], tgt[TS_MAXN * 2];
    float col[TS_MAXN * 3], tcol[TS_MAXN * 3];
    float seed[TS_MAXN], quant[TS_MAXN], rad[TS_MAXN];
    float alpha[TS_MAXN];          /* a drone's light, 0 .. 1 */
    float fade[TS_MAXN];           /* per second: > 0 fading in, < 0 leaving (gone at 0), 0 steady */
    int slot[TS_MAXN];             /* its place on its line */
    unsigned char line[TS_MAXN];   /* its line: 0 the first, 1 the second */
    int16_t head[TS_TABLE], next[TS_MAXN];   /* the hash's lists (drone numbers fit 16 bits: half the cache) */
    Ripple rip[TS_MAX_RIPPLES];
    int nrip;
    double time;                   /* the physics' clock (the wave phases) */
    double prev_beat;
    double cur_scale;              /* knob 1, eased */
    double hue;                    /* the hue drift's angle */
    double cam_dist, cam_cx, cam_cy;   /* render px per unit = 1 / cam_dist */
    int cam_init, scale_init;
    int fresh;                     /* the first text: placed at the size the knob asks for, at the next frame */
    uint32_t rng;                  /* mulberry32 */
    float t;                       /* the render clock (twinkle, stars) */
    int nstars;
    float star[TS_MAX_STARS * 4];  /* x, y (0..1), phase, brightness */
    float phys_ms, last_zoom;
} T;

/* the lines: what tx_line sampled, and where tx_layout put it */
typedef struct {
    int on, n, W, H;               /* sampled; its slots; its canvas */
    double len, sw, sh;            /* the outline's length (px); its size alone (the sim's: 4.5 tall, at most 16 wide) */
    double s;                      /* ... as units per px: exactly 4.5 / H (or 16 / W), so equal lines compare equal */
    int changed;                   /* new slots (or none) since the last layout */
    int shape;                     /* the first line is a shape (tx_shape): its slots are laid out already */
} TLine;
static TLine g_line[2];
static double g_lpts[2][TS_MAXN * 2];                        /* the slots: canvas px, as the sim's sampler leaves them */
static float g_ltgt[2][TS_MAXN * 2], g_lrgb[2][TS_MAXN * 3];  /* ... laid out: scene units, the rainbow */

int fx_version(void) { return TS_VERSION; }
int fx_param_count(void) { return P_COUNT; }
int fx_ready(void) { return T.ready; }
int fx_threads(int n);                                       /* (below: it waits for the physics' thread) */
void fx_trails_reset(void) { gl_trails_reset(); }
float fx_bench(int which, int n, float r) { return gl_bench(which, n, r); }

/* ---------------------------------------------------------------------------------------- the sim's helpers */
/* mulberry32 (engine/swarm.ts) */
static double ts_rand(void) {
    uint32_t a = (T.rng += 0x6d2b79f5u);
    uint32_t t = a;
    t = (t ^ (t >> 15)) * (t | 1u);
    t ^= t + (t ^ (t >> 7)) * (t | 61u);
    return (double)(t ^ (t >> 14)) / 4294967296.0;
}

/* three.js Color.setHSL (working space: the values as they are) */
static double ts_hue2rgb(double p, double q, double t) {
    if (t < 0.0) t += 1.0;
    if (t > 1.0) t -= 1.0;
    if (t < 1.0 / 6.0) return p + (q - p) * 6.0 * t;
    if (t < 1.0 / 2.0) return q;
    if (t < 2.0 / 3.0) return p + (q - p) * 6.0 * (2.0 / 3.0 - t);
    return p;
}
static void ts_hsl(double h, double s, double l, float *out) {
    h = h - floor(h);                                         /* euclideanModulo(h, 1) */
    const double p = l <= 0.5 ? l * (1.0 + s) : l + s - l * s, q = 2.0 * l - p;
    out[0] = (float)ts_hue2rgb(q, p, h + 1.0 / 3.0);
    out[1] = (float)ts_hue2rgb(q, p, h);
    out[2] = (float)ts_hue2rgb(q, p, h - 1.0 / 3.0);
}

/* the sim's spatial hash (its products are exact doubles there, then ToInt32: the same bits as these) */
static inline uint32_t ts_hash(int cx, int cy, int cz) {
    uint32_t h = (uint32_t)((int64_t)cx * 92837111LL) ^ (uint32_t)((int64_t)cy * 689287499LL) ^
                 (uint32_t)((int64_t)cz * 283923481LL);
    return h & TS_MASK;
}

static inline int ts_cell(double v) {                        /* floor, kept in range (hostile positions) */
    if (!(v > -1.0e6)) return -1000000;
    if (!(v < 1.0e6)) return 1000000;
    return (int)floor(v);
}
static inline int ts_cellc(double v) {                       /* the same without a branch (fmax(NaN, a) = a) */
    return (int)floor(fmin(fmax(v, -1.0e6), 1.0e6));
}

/* createSwarm: resting at (and targeting) a cloud, per-drone seeds and their rank quantiles */
static int g_qorder[TS_MAXN];
static int ts_cmp_seed(const void *a, const void *b) {
    int i = *(const int *)a, j = *(const int *)b;
    if (T.seed[i] != T.seed[j]) return T.seed[i] < T.seed[j] ? -1 : 1;
    return i - j;                                             /* the sim's sort is stable */
}
static void ts_create(const float *xy, const float *rgb, int n, uint32_t seed) {
    T.n = n;
    T.rng = seed;
    for (int i = 0; i < n; i++) {
        T.seed[i] = (float)ts_rand();
        g_qorder[i] = i;
    }
    qsort(g_qorder, (size_t)n, sizeof(int), ts_cmp_seed);
    for (int r = 0; r < n; r++) T.quant[g_qorder[r]] = (float)(((double)r + 0.5) / (double)n);
    memcpy(T.pos, xy, sizeof(float) * 2 * (size_t)n);
    memcpy(T.tgt, xy, sizeof(float) * 2 * (size_t)n);
    memset(T.vel, 0, sizeof(float) * 2 * (size_t)n);
    memcpy(T.col, rgb, sizeof(float) * 3 * (size_t)n);
    memcpy(T.tcol, rgb, sizeof(float) * 3 * (size_t)n);
    T.time = 0.0;
    T.nrip = 0;
    T.prev_beat = 0.0;
    for (int i = 0; i < n; i++) {                             /* (the lines' bookkeeping: all on the first line) */
        T.alpha[i] = 1.0f;
        T.fade[i] = 0.0f;
        T.slot[i] = i;
        T.line[i] = 0;
    }
}

/* the rank quantiles again (the swarm grew or shrank) */
static void ts_requant(void) {
    const int n = T.n;
    for (int i = 0; i < n; i++) g_qorder[i] = i;
    qsort(g_qorder, (size_t)n, sizeof(int), ts_cmp_seed);
    for (int r = 0; r < n; r++) T.quant[g_qorder[r]] = (float)(((double)r + 0.5) / (double)n);
}

/* assign(): untangle a new formation - both sets ranked by azimuth in the horizontal plane (the sim is y-up, so
 * atan2(z, x): a flat text has z = 0, which leaves the halves x >= 0 / x < 0) then height, rank to rank */
static const float *g_sort_xy;
static int ts_cmp_az(const void *a, const void *b) {
    int i = *(const int *)a, j = *(const int *)b;
    double ai = atan2(0.0, (double)g_sort_xy[i * 2]), aj = atan2(0.0, (double)g_sort_xy[j * 2]);
    if (ai != aj) return ai < aj ? -1 : 1;
    double yi = g_sort_xy[i * 2 + 1], yj = g_sort_xy[j * 2 + 1];
    if (yi != yj) return yi < yj ? -1 : 1;
    return i - j;
}
static int ts_cmp_x(const void *a, const void *b) {          /* across the text, left to right */
    int i = *(const int *)a, j = *(const int *)b;
    const float xi = g_sort_xy[i * 2], xj = g_sort_xy[j * 2];
    if (xi != xj) return xi < xj ? -1 : 1;
    return i - j;
}

/* ---------------------------------------------------------------------------------------- the physics */
typedef struct { double safe, max_speed, rigidity, avoidance, damping; } SwParams;
typedef struct {
    double level, bass, beat;
    double push_frac, push_amt;    /* push_amt <= 0: off */
    double waves_amt, ripples_amt; /* <= 0: off */
} SwWave;

/* the ripples of a step: a kick (the beat's rising edge) starts one at a random drone; old ones age out */
static void ts_ripples(double dt, const SwWave *w, int audioOn, int ripplesOn) {
    const int N = T.n;
    const double beat = w ? w->beat : 0.0;
    const int beatEdge = beat > 0.9 && T.prev_beat < 0.5;
    T.prev_beat = beat;
    if (audioOn && ripplesOn && beatEdge && N > 0) {
        int o = (int)(ts_rand() * (double)N);
        if (o >= N) o = N - 1;
        if (T.nrip >= TS_MAX_RIPPLES) {
            memmove(&T.rip[0], &T.rip[1], sizeof(Ripple) * (size_t)(T.nrip - 1));
            T.nrip--;
        }
        T.rip[T.nrip++] = (Ripple){ T.pos[o * 2], T.pos[o * 2 + 1], 0.0, 1.6 * w->ripples_amt };
    }
    for (int r = T.nrip - 1; r >= 0; r--) {
        T.rip[r].age += dt;
        if (T.rip[r].age > TS_RIPPLE_LIFE) {
            memmove(&T.rip[r], &T.rip[r + 1], sizeof(Ripple) * (size_t)(T.nrip - 1 - r));
            T.nrip--;
        }
    }
}

/* stepSwarm, the sim's arithmetic: dt seconds; sc = the target space's scale; w may be NULL (no audio) */
static void ts_step(double dt, const SwParams *p, double sc, const SwWave *w) {
    const int N = T.n;
    float *pos = T.pos, *vel = T.vel, *rad = T.rad;
    T.time += dt;
    const double time = T.time;
    const double safe = p->safe > 1.0e-3 ? p->safe : 1.0e-3;
    const double maxSp = p->max_speed;
    const double damp = fmax(0.0, 1.0 - p->damping * dt);
    const double maxSp2 = maxSp * maxSp;
    const double colRate = fmin(dt * 2.5, 1.0);

    /* ---- the wave field: per-drone safe-radius multipliers ---- */
    const int pushOn = w && w->push_amt > 0.0, wavesOn = w && w->waves_amt > 0.0, ripplesOn = w && w->ripples_amt > 0.0;
    const int audioOn = (pushOn || wavesOn || ripplesOn) && w && (w->level > 0.004 || w->bass > 0.004 || w->beat > 0.004);
    ts_ripples(dt, w, audioOn, ripplesOn);
    double maxRad = safe;
    if (audioOn || T.nrip > 0) {                              /* (a Trigger's ripple plays in silence too) */
        const double wAmp = wavesOn ? w->bass * 0.9 * w->waves_amt : 0.0;
        const double pulse = pushOn ? w->level * 2.2 * w->push_amt : 0.0;
        const double pushFrac = pushOn ? w->push_frac : 0.0;
        for (int i = 0; i < N; i++) {
            const double x = pos[i * 2], y = pos[i * 2 + 1];
            double m = 1.0;
            if (T.quant[i] < pushFrac) m += pulse;
            if (wAmp != 0.0) {
                m += wAmp * sin((x * 0.81 + y * 0.49 + 0.0 * 0.28) * 1.5 - time * 4.2);
                m += wAmp * 0.8 * sin((x * -0.25 + y * 0.83 + 0.0 * -0.46) * 1.9 + time * 3.1);
            }
            for (int r = 0; r < T.nrip; r++) {
                const Ripple *rp = &T.rip[r];
                const double dx = x - rp->x, dy = y - rp->y;
                const double d = sqrt(dx * dx + dy * dy) - rp->age * TS_RIPPLE_SPEED;
                const double q = d / TS_RIPPLE_W;
                m += rp->gain * exp(-q * q) * (1.0 - rp->age / TS_RIPPLE_LIFE);
            }
            if (!(m >= 0.35)) m = 0.35;                       /* (NaN -> 0.35) */
            else if (m > 4.0) m = 4.0;
            const double ri = safe * m;
            rad[i] = (float)ri;
            if (ri > maxRad) maxRad = ri;
        }
    } else {
        for (int i = 0; i < N; i++) rad[i] = (float)safe;
    }

    /* ---- the hash, cells the size of the largest radius ---- */
    const double inv = 1.0 / maxRad;
    for (int h = 0; h < TS_TABLE; h++) T.head[h] = -1;
    for (int i = 0; i < N; i++) {
        const uint32_t h = ts_hash(ts_cell(pos[i * 2] * inv), ts_cell(pos[i * 2 + 1] * inv), 0);
        T.next[i] = T.head[h];
        T.head[h] = (int16_t)i;
    }

    const double rig = p->rigidity, avoid = p->avoidance;
    for (int i = 0; i < N; i++) {
        const int ix = i * 2, iy = ix + 1;
        const double px = pos[ix], py = pos[iy];
        const double radI = rad[i];
        double ax = (T.tgt[ix] * sc - px) * rig;
        double ay = (T.tgt[iy] * sc - py) * rig;
        const int cx = ts_cell(px * inv), cy = ts_cell(py * inv);
        /* ts_hash(cx + dx, cy + dy, dz) = A(cx + dx) ^ B(cy + dy) ^ C(dz): the three parts once, not per cell */
        uint32_t ha[3], hb[3];
        for (int k = 0; k < 3; k++) {
            ha[k] = (uint32_t)((int64_t)(cx + k - 1) * 92837111LL);
            hb[k] = (uint32_t)((int64_t)(cy + k - 1) * 689287499LL);
        }
        static const uint32_t hc[3] = { (uint32_t)(-283923481LL), 0u, 283923481u };
        int visited = 0;
        for (int dx = -1; dx <= 1 && visited < TS_MAX_NEIGH; dx++)
            for (int dy = -1; dy <= 1 && visited < TS_MAX_NEIGH; dy++)
                for (int dz = -1; dz <= 1 && visited < TS_MAX_NEIGH; dz++) {
                    int j = T.head[(ha[dx + 1] ^ hb[dy + 1] ^ hc[dz + 1]) & TS_MASK];
                    while (j >= 0 && visited < TS_MAX_NEIGH) {
                        if (j != i) {
                            visited++;
                            const double rx = px - pos[j * 2], ry = py - pos[j * 2 + 1];
                            const double d2 = rx * rx + ry * ry;
                            const double thresh = (radI + rad[j]) * 0.5;
                            if (d2 < thresh * thresh && d2 > 1.0e-9) {
                                const double dist = sqrt(d2);
                                const double f = ((thresh - dist) / thresh * avoid) / dist;
                                ax += rx * f;
                                ay += ry * f;
                            }
                        }
                        j = T.next[j];
                    }
                }
        double vx = (vel[ix] + ax * dt) * damp;
        double vy = (vel[iy] + ay * dt) * damp;
        const double v2 = vx * vx + vy * vy;
        if (v2 > maxSp2) {
            const double s = maxSp / sqrt(v2);
            vx *= s;
            vy *= s;
        }
        double nx = px + vx * dt, ny = py + vy * dt;
        if (!(fabs(nx) < 1.0e4 && fabs(ny) < 1.0e4)) {        /* (not in the sim: a hostile frame never strands one) */
            nx = T.tgt[ix] * sc;
            ny = T.tgt[iy] * sc;
            vx = vy = 0.0;
            if (!(fabs(nx) < 1.0e4 && fabs(ny) < 1.0e4)) nx = ny = 0.0;
        }
        vel[ix] = (float)vx;
        vel[iy] = (float)vy;
        pos[ix] = (float)nx;
        pos[iy] = (float)ny;
        for (int k = 0; k < 3; k++)                           /* (as a Float32Array's +=: one rounding, of the sum) */
            T.col[i * 3 + k] = (float)((double)T.col[i * 3 + k] + ((double)T.tcol[i * 3 + k] - T.col[i * 3 + k]) * colRate);
    }
}

/* ---------------------------------------------------------------------------------------- the fast path */
/* sin to ~4e-6: the angle reduced to [-pi, pi], folded to [-pi/2, pi/2], the series to x^9 */
static inline float ts_fsin(float a) {
    float t = a * 0.15915494309189535f;
    t -= floorf(t + 0.5f);
    float s = t * 6.283185307179586f;
    const float as = fabsf(s);
    s = copysignf(fminf(as, 3.14159265358979f - as), s);
    const float s2 = s * s;
    return s * (1.0f - s2 * (1.0f / 6.0f) * (1.0f - s2 * (1.0f / 20.0f) * (1.0f - s2 * (1.0f / 42.0f) *
                                                                         (1.0f - s2 * (1.0f / 72.0f)))));
}

/* e^x for x <= 0 to ~1e-5 (0 below -80): 2^(x log2 e), the fraction's power by its series to the 6th */
static inline float ts_fexp_neg(float x) {
    if (!(x > -80.0f)) return 0.0f;
    if (x > 0.0f) x = 0.0f;
    const float y = x * 1.4426950408889634f;
    const float n = floorf(y), f = y - n;
    float p = 1.5403530393381606e-4f;
    p = p * f + 1.3333558146428443e-3f;
    p = p * f + 9.6181291076284772e-3f;
    p = p * f + 5.5504108664821580e-2f;
    p = p * f + 2.4022650695910071e-1f;
    p = p * f + 6.9314718055994531e-1f;
    p = p * f + 1.0f;
    int32_t bits;
    memcpy(&bits, &p, sizeof bits);
    bits += (int32_t)n * (1 << 23);                           /* x 2^n (n >= -116: stays a normal float) */
    memcpy(&p, &bits, sizeof p);
    return p;
}

/* the fast path's copy of the sim's hash: the same slots, each slot's drones side by side - newest first, as the
 * sim's lists - with their positions and radii beside them (the step writes each drone's new place back, so its
 * neighbours read it as the sim's do), and a bit per slot for whether anything is in it (2 KB: most of a drone's 27
 * slots are empty, and that is answered without going out to the rest of memory) */
typedef struct { float x, y, r; int32_t i; } TSDrone;
static TSDrone g_sd[TS_MAXN];
static int16_t g_off[TS_TABLE + 1];                          /* slot s: g_sd[g_off[s] .. g_off[s + 1]) */
static uint32_t g_occ[TS_TABLE / 32];
static int g_cx[TS_MAXN], g_cy[TS_MAXN], g_slot[TS_MAXN], g_loc[TS_MAXN];

/* a drone's 27 slots in the sim's order (dx, dy, dz) */
static inline void ts_slots27(int i, int *slot) {
    static const uint32_t hc[3] = { (uint32_t)(-283923481LL), 0u, 283923481u };
    uint32_t ha[3], hb[3];
    for (int k = 0; k < 3; k++) {
        ha[k] = (uint32_t)(g_cx[i] + k - 1) * 92837111u;
        hb[k] = (uint32_t)(g_cy[i] + k - 1) * 689287499u;
    }
    for (int dx = 0, s = 0; dx < 3; dx++)
        for (int dy = 0; dy < 3; dy++)
            for (int dz = 0; dz < 3; dz++, s++) slot[s] = (int)((ha[dx] ^ hb[dy] ^ hc[dz]) & TS_MASK);
}

/* stepSwarm as the mode runs it (see the top): ts_step's steps, visits and order, in float */
static void ts_step_fast(double dt_d, const SwParams *p, double sc_d, const SwWave *w) {
    const int N = T.n;
    float *pos = T.pos, *vel = T.vel, *rad = T.rad;
    T.time += dt_d;
    const double time = T.time;
    const float dt = (float)dt_d, sc = (float)sc_d;
    const float safe = p->safe > 1.0e-3 ? (float)p->safe : 1.0e-3f;
    const float maxSp = (float)p->max_speed, maxSp2 = maxSp * maxSp;
    const float damp = (float)fmax(0.0, 1.0 - p->damping * dt_d);
    const float colRate = fminf(dt * 2.5f, 1.0f);

    const int pushOn = w && w->push_amt > 0.0, wavesOn = w && w->waves_amt > 0.0, ripplesOn = w && w->ripples_amt > 0.0;
    const int audioOn = (pushOn || wavesOn || ripplesOn) && w && (w->level > 0.004 || w->bass > 0.004 || w->beat > 0.004);
    ts_ripples(dt_d, w, audioOn, ripplesOn);
    float maxRad = safe;
    if (audioOn || T.nrip > 0) {
        const float wAmp = wavesOn ? (float)(w->bass * 0.9 * w->waves_amt) : 0.0f;
        const float pulse = pushOn ? (float)(w->level * 2.2 * w->push_amt) : 0.0f;
        const float pushFrac = pushOn ? (float)w->push_frac : 0.0f;
        /* sin(a - time 4.2) = sin(a - (time 4.2 mod 2 pi)): the phases reduced once, in double */
        const float ph1 = (float)fmod(time * 4.2, 6.283185307179586), ph2 = (float)fmod(time * 3.1, 6.283185307179586);
        const int nr = T.nrip;
        float rx[TS_MAX_RIPPLES], ry[TS_MAX_RIPPLES], rr[TS_MAX_RIPPLES], rg[TS_MAX_RIPPLES];
        for (int r = 0; r < nr; r++) {
            rx[r] = (float)T.rip[r].x;
            ry[r] = (float)T.rip[r].y;
            rr[r] = (float)(T.rip[r].age * TS_RIPPLE_SPEED);
            rg[r] = (float)(T.rip[r].gain * (1.0 - T.rip[r].age / TS_RIPPLE_LIFE));
        }
        const float invW = (float)(1.0 / TS_RIPPLE_W);
        for (int i = 0; i < N; i++) {
            const float x = pos[i * 2], y = pos[i * 2 + 1];
            float m = 1.0f;
            if (T.quant[i] < pushFrac) m += pulse;
            if (wAmp != 0.0f) {
                m += wAmp * ts_fsin((x * 0.81f + y * 0.49f) * 1.5f - ph1);
                m += wAmp * 0.8f * ts_fsin((x * -0.25f + y * 0.83f) * 1.9f + ph2);
            }
            for (int r = 0; r < nr; r++) {
                const float dx = x - rx[r], dy = y - ry[r];
                const float q = (sqrtf(dx * dx + dy * dy) - rr[r]) * invW, q2 = q * q;
                if (q2 < 30.0f) m += rg[r] * ts_fexp_neg(-q2);        /* (e^-30 ~ 1e-13: nothing) */
            }
            if (!(m >= 0.35f)) m = 0.35f;
            else if (m > 4.0f) m = 4.0f;
            const float ri = safe * m;
            rad[i] = ri;
            if (ri > maxRad) maxRad = ri;
        }
    } else {
        for (int i = 0; i < N; i++) rad[i] = safe;
    }

    /* the sim's hash, cells the size of the largest radius (the cells in double, as ts_step): counted per slot, then
     * each drone set down at its slot's end, one before the last: ascending drones, so each slot holds its drones
     * newest first, as the sim's list does */
    const double inv = 1.0 / (double)maxRad;
    memset(g_off, 0, sizeof g_off);
    memset(g_occ, 0, sizeof g_occ);
    for (int i = 0; i < N; i++) {
        const int cx = ts_cellc(pos[i * 2] * inv), cy = ts_cellc(pos[i * 2 + 1] * inv);
        const int sl = (int)ts_hash(cx, cy, 0);
        g_cx[i] = cx;
        g_cy[i] = cy;
        g_slot[i] = sl;
        g_off[sl]++;
        g_occ[sl >> 5] |= 1u << (sl & 31);
    }
    for (int sl = 0, acc = 0; sl < TS_TABLE; sl++) {
        acc += g_off[sl];
        g_off[sl] = (int16_t)acc;                             /* (each slot's end, for now) */
    }
    for (int i = 0; i < N; i++) {
        const int k = --g_off[g_slot[i]];
        g_sd[k] = (TSDrone){ pos[i * 2], pos[i * 2 + 1], rad[i], i };
        g_loc[i] = k;
    }
    g_off[TS_TABLE] = (int16_t)N;                             /* (now each slot's start; its end, the next one's) */

    const float rig = (float)p->rigidity, avoid = (float)p->avoidance;
    int occ[27], nocc = -1, ocx = 0, ocy = 0;                  /* the last cell's occupied slots, in the sim's order */
    for (int i = 0; i < N; i++) {
        const int ix = i * 2, iy = ix + 1;
        const float px = pos[ix], py = pos[iy], radI = rad[i];
        float ax = (T.tgt[ix] * sc - px) * rig;
        float ay = (T.tgt[iy] * sc - py) * rig;
        if (nocc < 0 || g_cx[i] != ocx || g_cy[i] != ocy) {   /* (drones in a row often share a cell) */
            int slot[27];
            ts_slots27(i, slot);
            nocc = 0;
            for (int k = 0; k < 27; k++)                      /* (an empty slot: the sim's list is empty too) */
                if ((g_occ[slot[k] >> 5] >> (slot[k] & 31)) & 1u) occ[nocc++] = slot[k];
            ocx = g_cx[i];
            ocy = g_cy[i];
        }
        int visited = 0;
        for (int o = 0; o < nocc && visited < TS_MAX_NEIGH; o++) {
            const int sl = occ[o];
            const int k1 = g_off[sl + 1];
            for (int k = g_off[sl]; k < k1 && visited < TS_MAX_NEIGH; k++) {
                const TSDrone *d = &g_sd[k];
                if (d->i == i) continue;
                visited++;
                const float rx = px - d->x, ry = py - d->y;
                const float d2 = rx * rx + ry * ry;
                const float thresh = (radI + d->r) * 0.5f;
                if (d2 < thresh * thresh && d2 > 1.0e-9f) {
                    const float dist = sqrtf(d2);
                    const float f = (thresh - dist) * avoid / (thresh * dist);
                    ax += rx * f;
                    ay += ry * f;
                }
            }
        }
        float vx = (vel[ix] + ax * dt) * damp;
        float vy = (vel[iy] + ay * dt) * damp;
        const float v2 = vx * vx + vy * vy;
        if (v2 > maxSp2) {
            const float s = maxSp / sqrtf(v2);
            vx *= s;
            vy *= s;
        }
        float nx = px + vx * dt, ny = py + vy * dt;
        if (!(fabsf(nx) + fabsf(ny) < 1.0e4f)) {               /* (one comparison: each costs a pipeline stall here) */
            nx = T.tgt[ix] * sc;
            ny = T.tgt[iy] * sc;
            vx = vy = 0.0f;
            if (!(fabsf(nx) + fabsf(ny) < 1.0e4f)) nx = ny = 0.0f;
        }
        vel[ix] = vx;
        vel[iy] = vy;
        pos[ix] = nx;
        pos[iy] = ny;
        g_sd[g_loc[i]].x = nx;                                  /* (its neighbours to come read it here) */
        g_sd[g_loc[i]].y = ny;
        float *c = T.col + i * 3;
        const float *tc = T.tcol + i * 3;
        c[0] += (tc[0] - c[0]) * colRate;
        c[1] += (tc[1] - c[1]) * colRate;
        c[2] += (tc[2] - c[2]) * colRate;
    }
}

/* ---------------------------------------------------------------------------------------- the physics' own core */
/* The step runs on a thread of its own (THREADS 2, the default): fx_frame hands it the step and draws the swarm as
 * the step before left it while this one runs on another core (the A53's others are free: the engine's thread takes
 * one, glow.h's final pass a second), so a frame costs the larger of the two, not their sum; the picture is one step
 * (a frame) behind the physics - as with THREADS 1, which runs the step in fx_frame after the copy it draws, so the
 * frames are the same either way. Whatever else touches the swarm - the fades, a new text, a Trigger's ripple, the
 * tests - first waits for the step under way (ts_wait). */
static struct {
    pthread_t th;
    pthread_mutex_t mu;
    pthread_cond_t cv;
    int state;                     /* 0 no thread yet, 1 running, -1 none (steps run in place) */
    int busy;                      /* a step under way */
    double dt, sc, ms;             /* its time step and scale; how long the last step took */
    SwParams sp;
    SwWave wv;
} TW = { .mu = PTHREAD_MUTEX_INITIALIZER, .cv = PTHREAD_COND_INITIALIZER };
static int ts_async = 1;

static void *ts_worker(void *arg) {
    (void)arg;
    pthread_mutex_lock(&TW.mu);
    for (;;) {
        while (!TW.busy) pthread_cond_wait(&TW.cv, &TW.mu);
        const double dt = TW.dt, sc = TW.sc;
        const SwParams sp = TW.sp;
        const SwWave wv = TW.wv;
        pthread_mutex_unlock(&TW.mu);
        const double t0 = gl_now_ms();
        ts_step_fast(dt, &sp, sc, &wv);
        const double el = gl_now_ms() - t0;
        pthread_mutex_lock(&TW.mu);
        TW.ms = el;
        TW.busy = 0;
        pthread_cond_broadcast(&TW.cv);
    }
    return NULL;
}

/* the step under way, if any, has finished: the swarm is this thread's to read and change */
static void ts_wait(void) {
    if (TW.state != 1) return;
    pthread_mutex_lock(&TW.mu);
    while (TW.busy) pthread_cond_wait(&TW.cv, &TW.mu);
    pthread_mutex_unlock(&TW.mu);
}

/* a step: handed to the physics' thread (back at once), or run here */
static void ts_go(double dt, const SwParams *sp, double sc, const SwWave *wv) {
    if (ts_async && TW.state == 0) {
        TW.state = pthread_create(&TW.th, NULL, ts_worker, NULL) == 0 ? 1 : -1;
        if (TW.state == 1) pthread_detach(TW.th);
    }
    if (!ts_async || TW.state != 1) {
        const double t0 = gl_now_ms();
        ts_step_fast(dt, sp, sc, wv);
        TW.ms = gl_now_ms() - t0;
        return;
    }
    pthread_mutex_lock(&TW.mu);
    TW.dt = dt;
    TW.sc = sc;
    TW.sp = *sp;
    TW.wv = *wv;
    TW.busy = 1;
    pthread_cond_broadcast(&TW.cv);
    pthread_mutex_unlock(&TW.mu);
}

/* ---------------------------------------------------------------------------------------- the text */
/* the sim's collectOutline: the outline of an RGBA picture of the text (alpha > 60 = inside) resampled into count
 * points (canvas px, doubles as the sim) in pts. count <= 0: as many as per_px per px of outline, 16 .. maxn.
 * Returns the count, or < 0 (no text, too large, out of memory; pts untouched then); *len_out = the outline's length */
static int ts_outline(const uint8_t *rgba, int W, int H, int stride, int count, double per_px, int maxn, double *pts,
                      double *len_out) {
    if (!rgba || !pts || W < 3 || H < 3 || W > 8192 || H > 2048 || (int64_t)W * H > 8000000 || stride < W * 4 ||
        maxn < 1 || count > maxn)
        return -1;
    const size_t NP = (size_t)W * (size_t)H;
    uint8_t *in = (uint8_t *)malloc(NP), *visited = (uint8_t *)calloc(NP, 1);
    int *edges = (int *)malloc(sizeof(int) * NP);
    int rc = -3;
    int *cstart = NULL, *clist = NULL, *chain_pt = NULL, *chain_at = NULL;
    double *cum = NULL;
    int *alloc_n = NULL;
    if (!in || !visited || !edges) goto out;
    for (int y = 0; y < H; y++)
        for (int x = 0; x < W; x++) in[(size_t)y * W + x] = rgba[(size_t)y * (size_t)stride + (size_t)x * 4 + 3] > 60;
    int ne = 0;
    for (int y = 0; y < H; y++)
        for (int x = 0; x < W; x++) {
            const size_t i = (size_t)y * W + x;
            if (!in[i]) continue;
            if (x == 0 || y == 0 || x == W - 1 || y == H - 1 || !in[i - 1] || !in[i + 1] || !in[i - W] || !in[i + W])
                edges[ne++] = (int)i;
        }
    if (ne == 0) { rc = -2; goto out; }
    /* the grid of 3 px cells, each cell's edges in raster order (the sim's Map of arrays) */
    const int CELL = 3, GW = W / CELL + 1, GH = H / CELL + 1;
    cstart = (int *)calloc((size_t)GW * GH + 1, sizeof(int));
    clist = (int *)malloc(sizeof(int) * (size_t)ne);
    if (!cstart || !clist) goto out;
    for (int e = 0; e < ne; e++) cstart[((edges[e] / W) / CELL) * GW + (edges[e] % W) / CELL + 1]++;
    for (int c = 0; c < GW * GH; c++) cstart[c + 1] += cstart[c];
    {
        int *fill = (int *)malloc(sizeof(int) * (size_t)GW * GH);
        if (!fill) goto out;
        memcpy(fill, cstart, sizeof(int) * (size_t)GW * GH);
        for (int e = 0; e < ne; e++) {
            const int c = ((edges[e] / W) / CELL) * GW + (edges[e] % W) / CELL;
            clist[fill[c]++] = edges[e];
        }
        free(fill);
    }
    /* chain: from each unvisited edge pixel (raster order), step to the nearest unvisited within 3 px */
    chain_pt = (int *)malloc(sizeof(int) * (size_t)ne);       /* every edge pixel lands in exactly one chain */
    chain_at = (int *)malloc(sizeof(int) * ((size_t)ne + 1)); /* chain c = chain_pt[chain_at[c] .. chain_at[c + 1]) */
    if (!chain_pt || !chain_at) goto out;
    int nch = 0, np = 0;
    for (int e = 0; e < ne; e++) {
        int cur = edges[e];
        if (visited[cur]) continue;
        chain_at[nch++] = np;
        while (cur >= 0) {
            visited[cur] = 1;
            chain_pt[np++] = cur;
            const int x = cur % W, y = cur / W, gx0 = x / CELL, gy0 = y / CELL;
            int best = -1, bd = 9 + 1;
            for (int gx = gx0 - 1; gx <= gx0 + 1; gx++) {
                if (gx < 0 || gx >= GW) continue;
                for (int gy = gy0 - 1; gy <= gy0 + 1; gy++) {
                    if (gy < 0 || gy >= GH) continue;
                    const int c = gy * GW + gx;
                    for (int k = cstart[c]; k < cstart[c + 1]; k++) {
                        const int j = clist[k];
                        if (visited[j]) continue;
                        const int ddx = j % W - x, ddy = j / W - y, d = ddx * ddx + ddy * ddy;
                        if (d < bd) { bd = d; best = j; }
                    }
                }
            }
            cur = bd <= 9 ? best : -1;
        }
    }
    chain_at[nch] = np;
    if (nch < 1) { rc = -2; goto out; }                       /* (edges always make a chain: for the compiler) */
    /* arc lengths; the drones shared out by length (Math.round), the remainder to the first largest share */
    cum = (double *)malloc(sizeof(double) * (size_t)np);
    alloc_n = (int *)malloc(sizeof(int) * (size_t)nch);
    if (!cum || !alloc_n) goto out;
    double total = 0.0;
    for (int c = 0; c < nch; c++) {
        double L = 0.0;
        for (int k = chain_at[c] + 1; k < chain_at[c + 1]; k++) {
            const double dx = chain_pt[k] % W - chain_pt[k - 1] % W, dy = chain_pt[k] / W - chain_pt[k - 1] / W;
            L += sqrt(dx * dx + dy * dy);
        }
        cum[c] = L > 1.0e-3 ? L : 1.0e-3;                    /* (cum[] doubles as the lengths here) */
        total += cum[c];
    }
    if (count <= 0) {                                         /* as many as the density asks */
        const double v = floor(total * per_px + 0.5);
        count = !(v >= 16.0) ? (maxn < 16 ? maxn : 16) : (v > (double)maxn ? maxn : (int)v);
    }
    int64_t sum = 0;
    int gi = 0;
    for (int c = 0; c < nch; c++) {
        const double v = floor((double)count * cum[c] / total + 0.5);
        alloc_n[c] = v < 1.0 ? 1 : (v > 1.0e8 ? 100000000 : (int)v);
        sum += alloc_n[c];
        if (alloc_n[c] > alloc_n[gi]) gi = c;
    }
    {
        int64_t a = (int64_t)alloc_n[gi] + ((int64_t)count - sum);
        alloc_n[gi] = a < 1 ? 1 : (int)a;
    }
    /* resample each chain evenly by arc length (a chain's points all go in; the list is cut at count after) */
    int nout = 0;
    for (int c = 0; c < nch && nout < count; c++) {
        const int m = alloc_n[c], a0 = chain_at[c], len = chain_at[c + 1] - a0;
        if (m <= 0) continue;
        if (len == 1) {
            for (int k = 0; k < m && nout < count; k++) {
                pts[nout * 2] = (double)(chain_pt[a0] % W);
                pts[nout * 2 + 1] = (double)(chain_pt[a0] / W);
                nout++;
            }
            continue;
        }
        /* the chain's cumulative length (into cum[], reused from this chain's start: earlier chains are done) */
        double *cc = cum + a0;
        cc[0] = 0.0;
        for (int k = 1; k < len; k++) {
            const double dx = chain_pt[a0 + k] % W - chain_pt[a0 + k - 1] % W,
                         dy = chain_pt[a0 + k] / W - chain_pt[a0 + k - 1] / W;
            cc[k] = cc[k - 1] + sqrt(dx * dx + dy * dy);
        }
        const double Lc = cc[len - 1];
        int seg = 1;
        for (int k = 0; k < m; k++) {
            const double t = (m == 1 ? 0.5 : (double)k / (double)m) * Lc;
            while (seg < len && cc[seg] < t) seg++;          /* t only grows: the sim's restart from 1 finds the same */
            const int s = seg >= len ? len - 1 : seg;
            const double a = cc[s - 1], b = cc[s];
            const double f = b > a ? (t - a) / (b - a) : 0.0;
            const int p0 = chain_pt[a0 + s - 1], p1 = chain_pt[a0 + s];
            if (nout < count) {
                pts[nout * 2] = (double)(p0 % W) + (double)(p1 % W - p0 % W) * f;
                pts[nout * 2 + 1] = (double)(p0 / W) + (double)(p1 / W - p0 / W) * f;
            }
            nout++;
        }
    }
    if (nout > count) nout = count;
    for (int k = nout; k < count; k++) {                      /* (the sim pads with the last point) */
        pts[k * 2] = nout ? pts[(nout - 1) * 2] : (double)W * 0.5;
        pts[k * 2 + 1] = nout ? pts[(nout - 1) * 2 + 1] : (double)H * 0.5;
    }
    if (len_out) *len_out = total;
    rc = count;
out:
    free(in); free(visited); free(edges); free(cstart); free(clist); free(chain_pt); free(chain_at); free(cum); free(alloc_n);
    return rc;
}

/* cloudFromImageData's size: 4.5 tall, at most 16 wide */
static void ts_size(int W, int H, double *sw_out, double *sh_out) {
    double sh = 4.5, sw = 4.5 * ((double)W / (double)H);
    if (sw > 16.0) {
        const double k = 16.0 / sw;
        sw *= k;
        sh *= k;
    }
    *sw_out = sw;
    *sh_out = sh;
}

/* the sim's sampler and mapping for one text: n slots into xy / rgb (centred, flat, its rainbow by x). Returns n, or < 0 */
static int ts_sample(const uint8_t *rgba, int W, int H, int stride, int count, float *xy, float *rgb) {
    if (count < 1) return -1;
    double *pts = (double *)malloc(sizeof(double) * 2 * (size_t)count);
    if (!pts) return -3;
    const int rc = ts_outline(rgba, W, H, stride, count, 0.0, count, pts, NULL);
    if (rc > 0) {
        double sw, sh;
        ts_size(W, H, &sw, &sh);
        for (int k = 0; k < count; k++) {
            const double fx = pts[k * 2], fy = pts[k * 2 + 1];
            const float x = (float)((fx / W - 0.5) * sw), y = (float)((0.5 - fy / H) * sh);
            xy[k * 2] = x;
            xy[k * 2 + 1] = y;
            ts_hsl(((double)x / sw + 0.5) * 0.8 + 0.02, 0.85, 0.58, rgb + k * 3);
        }
    }
    free(pts);
    return rc;
}

/* where the lines' slots go: one line as the sim (its own size, centred); two at the same scale - the smaller of the
 * two lines' own, so the letters match - the first above the second, their middles pitch_px apart (canvas px), the
 * pair centred, the rainbow across the wider line. One line gives exactly ts_sample's slots and colours */
static int ts_two(void) { return !g_line[0].shape && g_line[1].on && g_line[1].n > 0; }   /* two lines on screen */
static void ts_layout_slots(double pitch_px) {
    if (g_line[0].shape) return;                              /* (a shape: laid out by tx_shape, no second line) */
    const int two = ts_two();
    double k[2] = { 1.0, 1.0 }, off[2] = { 0.0, 0.0 };
    double wb = g_line[0].sw;
    if (two) {
        const double s0 = g_line[0].s, s1 = g_line[1].s, s = s0 < s1 ? s0 : s1;
        k[0] = s / s0;                                        /* (1 exactly for the line that sets the scale) */
        k[1] = s / s1;
        off[0] = 0.5 * pitch_px * s;
        off[1] = -0.5 * pitch_px * s;
        const double w0 = g_line[0].sw * k[0], w1 = g_line[1].sw * k[1];
        wb = w0 > w1 ? w0 : w1;
    }
    for (int L = 0; L < (two ? 2 : 1); L++) {
        const TLine *ln = &g_line[L];
        const double *pts = g_lpts[L];
        for (int j = 0; j < ln->n; j++) {
            const double fx = pts[j * 2], fy = pts[j * 2 + 1];
            const float x = (float)((fx / ln->W - 0.5) * ln->sw * k[L]);
            double yy = (0.5 - fy / ln->H) * ln->sh * k[L];
            if (two) yy += off[L];
            g_ltgt[L][j * 2] = x;
            g_ltgt[L][j * 2 + 1] = (float)yy;
            ts_hsl(((double)x / wb + 0.5) * 0.8 + 0.02, 0.85, 0.58, g_lrgb[L] + j * 3);
        }
    }
}

/* drones that have faded out (or, with drop_leaving, every drone still leaving) out of the arrays, the order kept */
static void ts_compact(int drop_leaving) {
    int w = 0;
    for (int i = 0; i < T.n; i++) {
        if (T.fade[i] < 0.0f && (drop_leaving || T.alpha[i] <= 0.0f)) continue;
        if (w != i) {
            memcpy(T.pos + w * 2, T.pos + i * 2, sizeof(float) * 2);
            memcpy(T.vel + w * 2, T.vel + i * 2, sizeof(float) * 2);
            memcpy(T.tgt + w * 2, T.tgt + i * 2, sizeof(float) * 2);
            memcpy(T.col + w * 3, T.col + i * 3, sizeof(float) * 3);
            memcpy(T.tcol + w * 3, T.tcol + i * 3, sizeof(float) * 3);
            T.seed[w] = T.seed[i];
            T.quant[w] = T.quant[i];
            T.rad[w] = T.rad[i];
            T.alpha[w] = T.alpha[i];
            T.fade[w] = T.fade[i];
            T.slot[w] = T.slot[i];
            T.line[w] = T.line[i];
        }
        w++;
    }
    if (w != T.n) {
        T.n = w;
        ts_requant();
    }
}

/* k new drones of line L, each born where one of the npar drones in par is (chosen across the text: the m-th new
 * drone from left to right starts at the parent that far across), still, dark, fading in. With slots: the m-th
 * takes the m-th of those slots from left to right (they pour straight into place); without, the untangle places
 * them after. Returns how many were made */
static int ts_spawn(int L, int *slots, int k, int *par, int npar) {
    if (k > TS_MAXN - T.n) k = TS_MAXN - T.n;
    if (k <= 0) return 0;
    if (npar > 0) {
        g_sort_xy = T.pos;
        qsort(par, (size_t)npar, sizeof(int), ts_cmp_x);
    }
    if (slots) {
        g_sort_xy = g_ltgt[L];
        qsort(slots, (size_t)k, sizeof(int), ts_cmp_x);
    }
    const float sc = (float)(T.scale_init ? T.cur_scale : 1.0);
    for (int m = 0; m < k; m++) {
        const int i = T.n++;
        const int s = slots ? slots[m] : -1;
        if (npar > 0) {
            const int pa = par[(int)(((double)m + 0.5) * (double)npar / (double)k)];
            memcpy(T.pos + i * 2, T.pos + pa * 2, sizeof(float) * 2);
            memcpy(T.col + i * 3, T.col + pa * 3, sizeof(float) * 3);
            memcpy(T.tgt + i * 2, T.tgt + pa * 2, sizeof(float) * 2);
            memcpy(T.tcol + i * 3, T.tcol + pa * 3, sizeof(float) * 3);
            T.rad[i] = T.rad[pa];
        } else {                                              /* (nobody to start from: at its place) */
            const int ss = s >= 0 ? s : 0;
            T.pos[i * 2] = g_ltgt[L][ss * 2] * sc;
            T.pos[i * 2 + 1] = g_ltgt[L][ss * 2 + 1] * sc;
            memcpy(T.col + i * 3, g_lrgb[L] + ss * 3, sizeof(float) * 3);
            memcpy(T.tgt + i * 2, g_ltgt[L] + ss * 2, sizeof(float) * 2);
            memcpy(T.tcol + i * 3, g_lrgb[L] + ss * 3, sizeof(float) * 3);
            T.rad[i] = T.rad[0];
        }
        if (s >= 0) {
            memcpy(T.tgt + i * 2, g_ltgt[L] + s * 2, sizeof(float) * 2);
            memcpy(T.tcol + i * 3, g_lrgb[L] + s * 3, sizeof(float) * 3);
        }
        T.vel[i * 2] = T.vel[i * 2 + 1] = 0.0f;
        T.seed[i] = (float)ts_rand();
        T.quant[i] = 0.5f;                                    /* (ranked again by the caller) */
        T.alpha[i] = 0.0f;
        T.fade[i] = (float)(1.0 / TS_FADE);
        T.slot[i] = s;
        T.line[i] = (unsigned char)L;
    }
    return k;
}

/* line L's drones to its new slots: the sim's untangle, rank to rank. A new line pours out of the first; more slots
 * than drones: new drones born among the line's own; fewer: the ones left over (evenly spread) fade out where they are */
static int g_rank_d[TS_MAXN], g_rank_s[TS_MAXN], g_par[TS_MAXN];
static unsigned char g_keep[TS_MAXN];
static void ts_assign_line(int L) {
    const int b = g_line[L].on ? g_line[L].n : 0;
    int a = 0;
    for (int i = 0; i < T.n; i++) a += T.line[i] == L && T.fade[i] >= 0.0f;
    if (b > a && T.n + (b - a) > TS_MAXN) ts_compact(1);     /* (no room while others fade: they go now) */
    a = 0;
    for (int i = 0; i < T.n; i++)
        if (T.line[i] == L && T.fade[i] >= 0.0f) g_rank_d[a++] = i;
    if (a == 0) {
        if (b == 0) return;
        int np = 0;
        for (int i = 0; i < T.n; i++)
            if (T.line[i] != L && T.fade[i] >= 0.0f) g_par[np++] = i;
        for (int s = 0; s < b; s++) g_rank_s[s] = s;
        ts_spawn(L, g_rank_s, b, g_par, np);
        return;
    }
    if (b > a) {
        memcpy(g_par, g_rank_d, sizeof(int) * (size_t)a);
        ts_spawn(L, NULL, b - a, g_par, a);
        a = 0;
        for (int i = 0; i < T.n; i++)
            if (T.line[i] == L && T.fade[i] >= 0.0f) g_rank_d[a++] = i;
    }
    g_sort_xy = T.pos;
    qsort(g_rank_d, (size_t)a, sizeof(int), ts_cmp_az);
    for (int r = 0; r < a; r++) g_keep[g_rank_d[r]] = 0;
    if (b > 0) {
        for (int s = 0; s < b; s++) g_rank_s[s] = s;
        g_sort_xy = g_ltgt[L];
        qsort(g_rank_s, (size_t)b, sizeof(int), ts_cmp_az);
        const int nb = b < a ? b : a;                         /* (= b, unless the swarm was full) */
        for (int r = 0; r < nb; r++) {
            const int d = g_rank_d[a == nb ? r : (int)(((double)r + 0.5) * (double)a / (double)nb)];
            const int s = g_rank_s[r];
            g_keep[d] = 1;
            T.slot[d] = s;
            memcpy(T.tgt + d * 2, g_ltgt[L] + s * 2, sizeof(float) * 2);
            memcpy(T.tcol + d * 3, g_lrgb[L] + s * 3, sizeof(float) * 3);
        }
    }
    for (int r = 0; r < a; r++) {
        const int d = g_rank_d[r];
        if (!g_keep[d]) {
            T.fade[d] = (float)(-1.0 / TS_FADE);
            T.slot[d] = -1;
        }
    }
}

/* the drones of line L to where the layout put their slots (the slots themselves unchanged) */
static void ts_targets(int L) {
    for (int i = 0; i < T.n; i++) {
        if (T.line[i] != L || T.fade[i] < 0.0f) continue;
        const int s = T.slot[i];
        if (s < 0 || s >= g_line[L].n) continue;
        memcpy(T.tgt + i * 2, g_ltgt[L] + s * 2, sizeof(float) * 2);
        memcpy(T.tcol + i * 3, g_lrgb[L] + s * 3, sizeof(float) * 3);
    }
}

/* a line's slots from a picture of it (RGBA, its alpha), for the next tx_layout. which 0: the first line, always the
 * mode's COUNT drones; which 1: the second line - count drones, or with count <= 0 as many as keep its dots as close
 * together as the first line's (at most twice the first line's); no picture: no second line. Returns the line's drone
 * count, or < 0 (the line is as it was) */
int tx_line(int which, const uint8_t *rgba, int W, int H, int stride, int count) {
    ts_wait();
    if (!T.ready) return -9;
    if (which < 0 || which > 1) return -1;
    TLine *ln = &g_line[which];
    if (which == 1 && !rgba) {
        if (ln->on) {
            ln->on = 0;
            ln->n = 0;
            ln->changed = 1;
        }
        return 0;
    }
    if (which == 1 && g_line[0].shape) return -1;            /* (no second line under a shape) */
    int want, maxn;
    double per_px = 0.0;
    if (which == 0) {
        want = maxn = T.n1;
    } else {
        maxn = TS_MAXN - T.n1;
        if (count > 0) {
            want = count > maxn ? maxn : count;
        } else {
            if (maxn > 2 * T.n1) maxn = 2 * T.n1;
            if (g_line[0].on && g_line[0].len > 0.0) {
                want = 0;
                per_px = (double)g_line[0].n / g_line[0].len;
            } else {
                want = T.n1 < maxn ? T.n1 : maxn;
            }
        }
    }
    static double tmp[TS_MAXN * 2];
    double len = 0.0;
    const int rc = ts_outline(rgba, W, H, stride, want, per_px, maxn, tmp, &len);
    if (rc < 0) return rc;
    if (ln->on && !ln->shape && ln->n == rc && ln->W == W && ln->H == H &&
        memcmp(tmp, g_lpts[which], sizeof(double) * 2 * (size_t)rc) == 0)
        return rc;                                            /* the same slots again: nothing moves */
    memcpy(g_lpts[which], tmp, sizeof(double) * 2 * (size_t)rc);
    ln->on = 1;
    ln->n = rc;
    ln->W = W;
    ln->H = H;
    ln->len = len;
    ts_size(W, H, &ln->sw, &ln->sh);
    ln->s = 4.5 * ((double)W / (double)H) > 16.0 ? 16.0 / W : 4.5 / H;   /* (ts_size's cap) */
    ln->changed = 1;
    ln->shape = 0;
    return rc;
}

/* formations.ts flowerOfLife: 19 circle outlines in the hex arrangement, the drones shared out evenly (the first
 * circles one more), each circle one hue, 0.5 .. 0.92 round the wheel; flat. Its arithmetic, in its order */
static void ts_flower(int n, float *xy, float *rgb) {
    const double rc = 0.62;
    double cxs[49], cys[49];
    int nc = 0;
    const double R2 = pow(2.05 * rc, 2.0);
    for (int q = -3; q <= 3; q++)
        for (int r = -3; r <= 3; r++) {
            const double cx = rc * (q + r * 0.5), cy = rc * (r * sqrt(3.0) / 2.0);
            if (cx * cx + cy * cy <= R2) {
                cxs[nc] = cx;
                cys[nc] = cy;
                nc++;
            }
        }
    const int base = n / nc, rem = n % nc;
    int idx = 0;
    for (int ci = 0; ci < nc; ci++) {
        const int m = base + (ci < rem ? 1 : 0);
        for (int k = 0; k < m && idx < n; k++) {
            const double a = ((double)k / m) * 3.141592653589793 * 2.0;
            xy[idx * 2] = (float)((cxs[ci] + cos(a) * rc) * 1.6);
            xy[idx * 2 + 1] = (float)((cys[ci] + sin(a) * rc) * 1.6);
            ts_hsl(0.5 + ((double)ci / nc) * 0.42, 0.85, 0.58, rgb + idx * 3);
            idx++;
        }
    }
    for (; idx < n; idx++) {                                  /* (as the sim: the last point again) */
        memcpy(xy + idx * 2, xy + (idx - 1) * 2, sizeof(float) * 2);
        memcpy(rgb + idx * 3, rgb + (idx - 1) * 3, sizeof(float) * 3);
    }
}

/* a shape from the sim's formations.ts instead of a text, for the next tx_layout: the first line's slots (the mode's
 * COUNT of them), no second line. kind 1 = the Flower of Life. Returns the drone count, or < 0 */
int tx_shape(int kind) {
    ts_wait();
    if (!T.ready) return -9;
    if (kind != 1) return -1;
    const int n = T.n1;
    static float xy[TS_MAXN * 2], rgb[TS_MAXN * 3];
    ts_flower(n, xy, rgb);
    TLine *ln = &g_line[0];
    if (!(ln->on && ln->shape == kind && ln->n == n && memcmp(xy, g_ltgt[0], sizeof(float) * 2 * (size_t)n) == 0 &&
          memcmp(rgb, g_lrgb[0], sizeof(float) * 3 * (size_t)n) == 0)) {   /* (the same shape again: nothing moves) */
        memcpy(g_ltgt[0], xy, sizeof(float) * 2 * (size_t)n);
        memcpy(g_lrgb[0], rgb, sizeof(float) * 3 * (size_t)n);
        ln->on = 1;
        ln->n = n;
        ln->W = ln->H = 0;
        ln->len = 0.0;
        ln->sw = ln->sh = ln->s = 0.0;
        ln->changed = 1;
        ln->shape = kind;
    }
    if (g_line[1].on) {
        g_line[1].on = 0;
        g_line[1].n = 0;
        g_line[1].changed = 1;
    }
    return n;
}

/* for the tests: the sim's Flower of Life itself (formations.ts), n points */
int tx_flower(int n, float *xy, float *rgb) {
    if (!xy || !rgb || n < 1 || n > TS_MAXN) return -1;
    ts_flower(n, xy, rgb);
    return n;
}

/* the lines laid out (pitch_px: from the first line's middle to the second's, canvas px) and the drones sent to them:
 * the first text places them (as the sim mounts); later a changed line's drones fly to its new slots and the others
 * to where the layout moved theirs. Returns the drone count, or < 0 */
static float g_xy[TS_MAXN * 2], g_rgb[TS_MAXN * 3];
int tx_layout(double pitch_px) {
    ts_wait();
    if (!T.ready) return -9;
    if (!g_line[0].on || g_line[0].n < 1) return -2;
    if (!(pitch_px >= 0.0)) pitch_px = 0.0;
    if (pitch_px > 4.0 * g_line[0].H) pitch_px = 4.0 * g_line[0].H;
    ts_layout_slots(pitch_px);
    const int two = ts_two();
    if (!T.have_text) {
        const int n0 = g_line[0].n, n = n0 + (two ? g_line[1].n : 0);
        memcpy(g_xy, g_ltgt[0], sizeof(float) * 2 * (size_t)n0);
        memcpy(g_rgb, g_lrgb[0], sizeof(float) * 3 * (size_t)n0);
        if (two) {
            memcpy(g_xy + 2 * n0, g_ltgt[1], sizeof(float) * 2 * (size_t)g_line[1].n);
            memcpy(g_rgb + 3 * n0, g_lrgb[1], sizeof(float) * 3 * (size_t)g_line[1].n);
        }
        ts_create(g_xy, g_rgb, n, T.rng ? T.rng : 7u);
        for (int i = n0; i < n; i++) {
            T.line[i] = 1;
            T.slot[i] = i - n0;
        }
        T.have_text = 1;
        T.fresh = 1;
    } else {
        const int n_before = T.n;
        for (int L = 0; L < 2; L++) {
            if (g_line[L].changed) ts_assign_line(L);
            else if (L == 0 || two) ts_targets(L);
        }
        if (T.n != n_before) ts_requant();
    }
    g_line[0].changed = g_line[1].changed = 0;
    return T.n;
}

/* one line, the old way (and for the tests): the mode's text, no second line */
int tx_text(const uint8_t *rgba, int W, int H, int stride) {
    const int rc = tx_line(0, rgba, W, H, stride, 0);
    if (rc < 0) return rc;
    tx_line(1, NULL, 0, 0, 0, 0);
    const int n = tx_layout(0.0);
    return n < 0 ? n : rc;
}

/* for the tests: the slots tx_text would make (no swarm needed) */
int tx_sample(const uint8_t *rgba, int W, int H, int stride, int count, float *xy, float *rgb) {
    if (!xy || !rgb || count < 1 || count > TS_MAXN) return -1;
    return ts_sample(rgba, W, H, stride, count, xy, rgb);
}

/* for the tests: a swarm resting at a cloud (createSwarm), one physics step (stepSwarm: the sim's arithmetic, or the
 * mode's), the state */
int tx_set_cloud(const float *xy, const float *rgb, int n, uint32_t seed) {
    if (!xy || !rgb || n < 1 || n > TS_MAXN) return -1;
    ts_wait();
    ts_create(xy, rgb, n, seed);
    T.have_text = 1;
    T.ready = 1;
    return n;
}
static int ts_physics(double dt, double sc, const double *prm, const double *wave, int fast) {
    if (!T.ready || !prm) return -1;
    ts_wait();
    SwParams p = { prm[0], prm[1], prm[2], prm[3], prm[4] };
    SwWave w;
    if (wave) {
        w = (SwWave){ wave[0], wave[1], wave[2], wave[3], wave[4], wave[5], wave[6] };
    }
    if (fast) ts_step_fast(dt, &p, sc, wave ? &w : NULL);
    else ts_step(dt, &p, sc, wave ? &w : NULL);
    return 0;
}
int tx_physics(double dt, double sc, const double *prm, const double *wave) { return ts_physics(dt, sc, prm, wave, 0); }
int tx_physics_fast(double dt, double sc, const double *prm, const double *wave) {
    return ts_physics(dt, sc, prm, wave, 1);
}
int tx_seeds(float *seed, float *quant, int n) {
    ts_wait();
    if (n < 0 || n > T.n) return -1;
    if (seed) memcpy(seed, T.seed, sizeof(float) * (size_t)n);
    if (quant) memcpy(quant, T.quant, sizeof(float) * (size_t)n);
    return T.n;
}
int tx_get(float *pos, float *vel, float *col, float *rad, int n) {
    ts_wait();
    if (n < 0 || n > T.n) return -1;
    if (pos) memcpy(pos, T.pos, sizeof(float) * 2 * (size_t)n);
    if (vel) memcpy(vel, T.vel, sizeof(float) * 2 * (size_t)n);
    if (col) memcpy(col, T.col, sizeof(float) * 3 * (size_t)n);
    if (rad) memcpy(rad, T.rad, sizeof(float) * (size_t)n);
    return T.n;
}
/* for the tests: the lines' bookkeeping - per drone its target, light, fade, line, slot (any may be NULL) */
int tx_get_lines(float *tgt, float *alpha, float *fade, unsigned char *line, int *slot, int n) {
    ts_wait();
    if (n < 0 || n > T.n) return -1;
    if (tgt) memcpy(tgt, T.tgt, sizeof(float) * 2 * (size_t)n);
    if (alpha) memcpy(alpha, T.alpha, sizeof(float) * (size_t)n);
    if (fade) memcpy(fade, T.fade, sizeof(float) * (size_t)n);
    if (line) memcpy(line, T.line, (size_t)n);
    if (slot) memcpy(slot, T.slot, sizeof(int) * (size_t)n);
    return T.n;
}
/* for the tests: drones; the first line's; the second line on, its slots; its outline length and the first's (px) */
int tx_info(double *out, int n) {
    ts_wait();
    double v[6] = { (double)T.n, (double)T.n1, (double)g_line[1].on, (double)g_line[1].n, g_line[1].len, g_line[0].len };
    for (int i = 0; i < n && i < 6; i++) out[i] = v[i];
    return T.n;
}
/* for the tests: a line's slots as laid out (units) and their colours; returns how many */
int tx_line_slots(int which, float *xy, float *rgb, int n) {
    ts_wait();
    if (which < 0 || which > 1 || n < 0) return -1;
    const int m = g_line[which].on ? g_line[which].n : 0;
    if (n > m) n = m;
    if (xy) memcpy(xy, g_ltgt[which], sizeof(float) * 2 * (size_t)n);
    if (rgb) memcpy(rgb, g_lrgb[which], sizeof(float) * 3 * (size_t)n);
    return m;
}

/* ---------------------------------------------------------------------------------------- set-up */
int fx_threads(int n) {                                       /* 2: the final pass on two cores, the physics on a third */
    ts_wait();
    ts_async = n >= 2;
    return gl_set_threads(n);
}

int fx_init(int n, int seed) {
    ts_wait();
    memset(&T, 0, sizeof T);
    memset(g_line, 0, sizeof g_line);
    n = n < 16 ? 16 : (n > TS_MAXN / 2 ? TS_MAXN / 2 : n);   /* (half the room: the other half for a second line) */
    T.n1 = n;
    gl_rng_state = 0x9E3779B9u ^ ((uint32_t)seed * 2654435761u);
    if (!gl_rng_state) gl_rng_state = 1u;
    /* until the mode's text arrives: a word-sized loop, coloured as the text will be */
    for (int i = 0; i < n; i++) {
        double th = (double)i / n * 6.283185307179586;
        g_xy[i * 2] = (float)(6.0 * cos(th));
        g_xy[i * 2 + 1] = (float)(1.2 * sin(th));
        ts_hsl(((double)g_xy[i * 2] / 14.0 + 0.5) * 0.8 + 0.02, 0.85, 0.58, g_rgb + i * 3);
    }
    ts_create(g_xy, g_rgb, n, (uint32_t)seed * 2654435761u + 1u);
    for (int s = 0; s < TS_MAX_STARS; s++) {
        T.star[s * 4] = gl_rnd();
        T.star[s * 4 + 1] = gl_rnd();
        T.star[s * 4 + 2] = gl_rnd() * 6.2831853f;
        float b = gl_rnd();
        T.star[s * 4 + 3] = 0.05f + 0.30f * b * b;          /* most faint, a few brighter */
    }
    T.ready = 1;
    return 0;
}

/* ---------------------------------------------------------------------------------------- a frame */
static float g_dpos[TS_MAXN * 2], g_dcol[TS_MAXN * 3], g_dalpha[TS_MAXN], g_dseed[TS_MAXN];   /* what is drawn */
int fx_frame(uint8_t *dst, int dW, int dH, int dpitch, int down, int sh_r, int sh_g, int sh_b, float *acc, int W, int H,
             float *bloom, int BW, int BH, const float *fft, int nfft, float fft_hz, const float *wave, int nwave,
             const float *bands, int nbands, const float *P) {
    (void)fft_hz;
    if (!T.ready) return -9;
    if (!P) return -1;
    if (nfft < 0 || nfft > 65536 || nwave < 0 || nwave > 65536 || nbands < 0 || nbands > 256 || (nfft > 0 && !fft) ||
        (nwave > 0 && !wave) || (nbands > 0 && !bands))
        return -3;
    const float k3 = gl_c01(P[P_K3]);
    const float bloom_amt = k3 < 0.5f ? k3 * 2.0f * 1.4f : 1.4f;   /* the Swarm's knob 3: bloom, then trails from 0.6 */
    const float decay = k3 >= 0.6f ? 0.25f + (k3 - 0.6f) * (0.63f / 0.4f) : 0.0f;
    int rc = gl_begin(dst, dW, dH, dpitch, down, sh_r, sh_g, sh_b, acc, W, H, bloom, BW, BH, bloom_amt > 0.0f);
    if (rc) return rc;
    const float dtf = gl_clampf(P[P_DT], 0.0f, 0.1f);
    T.t += dtf;
    if (T.t > 1.0e5f) T.t = 0.0f;
    const double dt = dtf < 1.0f / 30.0f ? dtf : 1.0 / 30.0;     /* the sim: dt <= 1/30 */
    ts_wait();                                                /* (the step handed over last frame: done) */
    T.phys_ms = (float)TW.ms;

    /* drones fading in or out; the ones gone out of the swarm */
    int gone = 0;
    for (int i = 0; i < T.n; i++) {
        const float f = T.fade[i];
        if (f == 0.0f) continue;
        float a = T.alpha[i] + f * dtf;
        if (f > 0.0f && a >= 1.0f) {
            a = 1.0f;
            T.fade[i] = 0.0f;
        } else if (f < 0.0f && a <= 0.0f) {
            a = 0.0f;
            gone++;
        }
        T.alpha[i] = a;
    }
    if (gone) ts_compact(0);

    /* the audio, through the intensity (the sim's gain), and the rack's amounts */
    const float gain = gl_clampf(P[P_INTENSITY], 0.0f, 10.0f);
    const float level = gl_c01(P[P_LEVEL] * gain), bass = gl_c01(P[P_BASS] * gain), mid = gl_c01(P[P_MID] * gain),
                treble = gl_c01(P[P_TREBLE] * gain), beat = gl_c01(P[P_BEAT]);
    const float pulse = gl_clampf(P[P_PULSE], 0.0f, 10.0f), swell = gl_clampf(P[P_SWELL], 0.0f, 10.0f);
    const float shim = gl_clampf(P[P_SHIMMER], 0.0f, 10.0f), drift = gl_clampf(P[P_HUE_DRIFT], 0.0f, 10.0f);
    const float uLevel = level * pulse, uBeat = beat * pulse, shimmer = treble * shim;
    if (drift > 0.0f) {
        T.hue += (double)mid * drift * dt * 0.6;
        if (T.hue > 1.0e4) T.hue -= 6.283185307179586 * 1000.0;
    }

    /* knob 1: the size of the target space, eased (k 8); bass swell breathes it */
    const float smin = gl_clampf(P[P_SIZE_MIN], 0.05f, 20.0f), smax = gl_clampf(P[P_SIZE_MAX], 0.05f, 20.0f);
    const double size_target = smin + (smax - smin) * gl_c01(P[P_K1]);
    if (!T.scale_init) { T.cur_scale = size_target; T.scale_init = 1; }
    T.cur_scale += (size_target - T.cur_scale) * fmin(1.0, dt * 8.0);
    const double sc = T.cur_scale * (1.0 + (double)bass * 0.1 * swell);
    if (T.fresh) {                                            /* (the sim mounts at 1x and lets Size spread it: here the
                                                                 text is simply there when the mode is first shown) */
        for (int i = 0; i < T.n * 2; i++) T.pos[i] = (float)(T.pos[i] * sc);
        T.fresh = 0;
    }

    /* the physics, in scene units */
    SwParams sp;
    sp.safe = gl_clampf(P[P_SAFE], 0.05f, 20.0f) * TS_LU;
    sp.max_speed = gl_clampf(P[P_SPEED], 0.1f, 1000.0f) * TS_LU;
    sp.rigidity = gl_clampf(P[P_RIGID], 0.0f, 100.0f);
    sp.avoidance = gl_clampf(P[P_AVOID], 0.0f, 1000.0f);
    sp.damping = gl_clampf(P[P_DAMP], 0.0f, 50.0f);
    SwWave wv;
    wv.level = level; wv.bass = bass; wv.beat = beat;
    wv.push_amt = gl_clampf(P[P_PUSH], 0.0f, 10.0f);
    wv.push_frac = gl_c01(P[P_PUSH_FRAC]);
    wv.waves_amt = gl_c01(P[P_K2]) * gl_clampf(P[P_WAVES_MAX], 0.0f, 10.0f);
    wv.ripples_amt = gl_clampf(P[P_RIPPLES], 0.0f, 10.0f);
    if (P[P_TRIG] > 0.5f) {                                   /* the performer's Trigger: a ripple out of the middle */
        if (T.nrip >= TS_MAX_RIPPLES) {
            memmove(&T.rip[0], &T.rip[1], sizeof(Ripple) * (size_t)(T.nrip - 1));
            T.nrip--;
        }
        T.rip[T.nrip++] = (Ripple){ 0.0, 0.0, 0.0, TS_TRIG_GAIN };
    }
    /* the picture: the swarm as the last step left it (the same frames with THREADS 1 or 2); then this frame's step -
     * on the physics' thread, meanwhile, or here */
    const int N = T.n;
    memcpy(g_dpos, T.pos, sizeof(float) * 2 * (size_t)N);
    memcpy(g_dcol, T.col, sizeof(float) * 3 * (size_t)N);
    memcpy(g_dalpha, T.alpha, sizeof(float) * (size_t)N);
    memcpy(g_dseed, T.seed, sizeof(float) * (size_t)N);
    ts_go(dt, &sp, sc, &wv);

    /* the view: the swarm's box, fitted to the render (full width), eased as the sim's Constrain view */
    double minx = 1.0e30, maxx = -1.0e30, miny = 1.0e30, maxy = -1.0e30;
    for (int i = 0; i < N; i++) {
        const double x = g_dpos[i * 2], y = g_dpos[i * 2 + 1];
        if (x < minx) minx = x;
        if (x > maxx) maxx = x;
        if (y < miny) miny = y;
        if (y > maxy) maxy = y;
    }
    const float Wr = (float)GF.W, Hr = (float)GF.H;
    const double dot_w = gl_clampf(P[P_DOT], 0.01f, 10.0f) * TS_LU;   /* a dot's radius, scene units */
    if (maxx >= minx && maxy >= miny && maxx - minx < 1.0e5 && maxy - miny < 1.0e5) {
        const double bw = (maxx - minx) + 4.0 * dot_w, bh = (maxy - miny) + 4.0 * dot_w;
        const double zoom = fmin(Wr * 0.94 / fmax(bw, 1.0e-3), Hr * 0.86 / fmax(bh, 1.0e-3));
        const double tdist = 1.0 / zoom, tcx = 0.5 * (minx + maxx), tcy = 0.5 * (miny + maxy);
        if (!T.cam_init) {
            T.cam_dist = tdist; T.cam_cx = tcx; T.cam_cy = tcy;
            T.cam_init = 1;
        } else {
            const double kk = tdist > T.cam_dist ? 6.0 : 1.6;   /* fast out, lazy in */
            T.cam_dist += (tdist - T.cam_dist) * fmin(1.0, dt * kk);
            T.cam_cx += (tcx - T.cam_cx) * fmin(1.0, dt * 2.5);
            T.cam_cy += (tcy - T.cam_cy) * fmin(1.0, dt * 2.5);
        }
    }
    const float zoom = T.cam_dist > 0.0 ? (float)(1.0 / T.cam_dist) : 1.0f;
    T.last_zoom = zoom;

    /* the stars: faint, still, twinkling slowly */
    int ns = (int)gl_clampf(P[P_STARS], 0.0f, (float)TS_MAX_STARS);
    const float res = Hr / 720.0f;
    for (int s = 0; s < ns; s++) {
        float b = T.star[s * 4 + 3] * (0.75f + 0.25f * gl_sin(T.t * 0.7f + T.star[s * 4 + 2]));
        gl_sprite(T.star[s * 4] * Wr, T.star[s * 4 + 1] * Hr, 0.8f * fmaxf(1.0f, res), b, b, b);
    }

    /* the drones: the sim's player.vert + drone.frag */
    const float audio_size = 1.0f + uBeat * 0.55f + uLevel * 0.22f;
    const float rmin = gl_clampf(P[P_DOT_MIN], 0.0f, 10.0f) * res;
    float r = (float)(dot_w * zoom) * audio_size;
    r = r < rmin ? rmin : r;
    const float amp0 = gl_clampf(P[P_BRIGHT], 0.0f, 20.0f) * (1.0f + uLevel * 0.6f + uBeat * 0.5f) * 1.5f;
    const float ang = (float)(gl_c01(P[P_K4]) * 6.283185307179586 + T.hue);
    const int turning = ang != 0.0f;
    const float hc = cosf(ang), hs = sinf(ang), hk = 0.57735026919f;
    const float tw_rate = 2.5f + shimmer * 10.0f, tw_amp = 0.1f + 0.55f * shimmer;
    const float cxs = Wr * 0.5f, cys = Hr * 0.5f;
    const float ccx = (float)T.cam_cx, ccy = (float)T.cam_cy;
    for (int i = 0; i < N; i++) {
        const float al = g_dalpha[i];
        if (!(al > 0.0f)) continue;
        const float sx = cxs + (g_dpos[i * 2] - ccx) * zoom, sy = cys - (g_dpos[i * 2 + 1] - ccy) * zoom;
        float cr = g_dcol[i * 3], cg = g_dcol[i * 3 + 1], cb = g_dcol[i * 3 + 2];
        if (turning) {                                         /* Rodrigues about (1,1,1)/sqrt(3) */
            const float dot = hk * (cr + cg + cb);
            const float kxr = hk * (cb - cg), kxg = hk * (cr - cb), kxb = hk * (cg - cr);
            const float nr = cr * hc + kxr * hs + hk * dot * (1.0f - hc);
            const float ng = cg * hc + kxg * hs + hk * dot * (1.0f - hc);
            const float nb = cb * hc + kxb * hs + hk * dot * (1.0f - hc);
            cr = nr; cg = ng; cb = nb;
        }
        const float tw = 0.9f + tw_amp * gl_sin(T.t * tw_rate + g_dseed[i] * 120.0f);
        const float a = amp0 * tw * al;
        gl_sprite(sx, sy, r, a * fmaxf(cr, 0.0f), a * fmaxf(cg, 0.0f), a * fmaxf(cb, 0.0f));
    }
    const float bg[3] = { gl_c01(P[P_BG_R]), gl_c01(P[P_BG_G]), gl_c01(P[P_BG_B]) };
    gl_end(bloom_amt, decay, 1.0f, bg, P[P_PREFILLED] > 0.5f);
    return 0;
}

/* for tests and DEBUG: glow's stats, then the physics' ms, the zoom (px per unit), the drones */
void fx_stats(float *out, int n) {
    float v[11];
    gl_stats(v);
    v[8] = T.phys_ms;
    v[9] = T.last_zoom;
    v[10] = (float)T.n;
    for (int i = 0; i < n && i < 11; i++) out[i] = v[i];
}
