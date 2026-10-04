/* textswarm.c - the kernel of the EYESY mode "14 - Swarm - Text": free.vet's swarm sim writing a word in drones -
 * FREE.VET unless the mode is told otherwise - with the sim's neighbor physics and its audio-FX rack, in the Swarm
 * Visualizer's light (glow.h).
 *
 * The text: the mode renders it with pygame in its font at 220 px with 60 px of padding (the sim's canvas) and hands
 * the pixels to tx_text(), which turns them into drone slots exactly as the sim's sources.ts does (sampleText /
 * cloudFromImageData / collectOutline): a pixel is inside if its alpha > 60; the boundary pixels are chained nearest
 * first (within 3 px); every chain gets its share of the drones by length and is resampled evenly by arc length; the
 * text is 4.5 units tall (at most 16 wide), flat; each slot is coloured with the sim's rainbow by x,
 * hsl(u 0.8 + 0.02, 0.85, 0.58). A new text: the drones fly to it, the sim's untangle (rank by azimuth, then height).
 *
 * The physics: engine/swarm.ts stepSwarm, line by line, on the live sim's path (no acceleration, jerk or downwash
 * limits): every drone springs toward its slot (x the swarm's scale) and pushes off any neighbour inside the mean of
 * the two drones' safe radii, f = (thresh - d) / thresh x avoidance; velocity damped by 1 - damping dt, clamped to the
 * speed limit; drones updated in place in order (as the sim); neighbours from a spatial hash (1 << 14 cells of the
 * largest radius, the 27 around a drone, at most 48 visits). Arithmetic in double, state in float: as the sim
 * (JavaScript numbers, Float32Arrays). Units: the mode passes light units (lu); 1 lu = 0.08 scene units (the sim's
 * LIGHT_UNIT). The text is flat, so z stays 0 and is not stored (the hash still walks the sim's 27 cells).
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

#define TS_VERSION 1
#define TS_MAXN 8192
#define TS_TABLE (1 << 14)
#define TS_MASK (TS_TABLE - 1)
#define TS_MAX_NEIGH 48
#define TS_MAX_RIPPLES 4
#define TS_RIPPLE_LIFE 1.8
#define TS_RIPPLE_SPEED 7.0
#define TS_RIPPLE_W 0.9
#define TS_LU 0.08                 /* scene units per light unit (the sim's LIGHT_UNIT) */
#define TS_MAX_STARS 600
#define TS_TRIG_GAIN 3.2           /* a Trigger's ripple: twice a kick's at the sim's Ripples 1.0 */

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
    float pos[TS_MAXN * 2], vel[TS_MAXN * 2], tgt[TS_MAXN * 2];
    float col[TS_MAXN * 3], tcol[TS_MAXN * 3];
    float seed[TS_MAXN], quant[TS_MAXN], rad[TS_MAXN];
    int head[TS_TABLE], next[TS_MAXN];
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

int fx_version(void) { return TS_VERSION; }
int fx_param_count(void) { return P_COUNT; }
int fx_ready(void) { return T.ready; }
int fx_threads(int n) { return gl_set_threads(n); }
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
}

/* assign(): untangle a new formation - both sets ranked by azimuth in the horizontal plane (the sim is y-up, so
 * atan2(z, x): a flat text has z = 0, which leaves the halves x >= 0 / x < 0) then height, rank to rank */
static int g_cur[TS_MAXN], g_tar[TS_MAXN];
static const float *g_sort_xy;
static int ts_cmp_az(const void *a, const void *b) {
    int i = *(const int *)a, j = *(const int *)b;
    double ai = atan2(0.0, (double)g_sort_xy[i * 2]), aj = atan2(0.0, (double)g_sort_xy[j * 2]);
    if (ai != aj) return ai < aj ? -1 : 1;
    double yi = g_sort_xy[i * 2 + 1], yj = g_sort_xy[j * 2 + 1];
    if (yi != yj) return yi < yj ? -1 : 1;
    return i - j;
}
static void ts_retarget(const float *xy, const float *rgb) {
    const int n = T.n;
    for (int i = 0; i < n; i++) { g_cur[i] = i; g_tar[i] = i; }
    g_sort_xy = T.pos;
    qsort(g_cur, (size_t)n, sizeof(int), ts_cmp_az);
    g_sort_xy = xy;
    qsort(g_tar, (size_t)n, sizeof(int), ts_cmp_az);
    for (int r = 0; r < n; r++) {
        const int d = g_cur[r], t = g_tar[r];
        T.tgt[d * 2] = xy[t * 2];
        T.tgt[d * 2 + 1] = xy[t * 2 + 1];
        T.tcol[d * 3] = rgb[t * 3];
        T.tcol[d * 3 + 1] = rgb[t * 3 + 1];
        T.tcol[d * 3 + 2] = rgb[t * 3 + 2];
    }
}

/* ---------------------------------------------------------------------------------------- the physics */
typedef struct { double safe, max_speed, rigidity, avoidance, damping; } SwParams;
typedef struct {
    double level, bass, beat;
    double push_frac, push_amt;    /* push_amt <= 0: off */
    double waves_amt, ripples_amt; /* <= 0: off */
} SwWave;

/* stepSwarm: dt seconds; sc = the target space's scale; w may be NULL (no audio) */
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
        T.rip[T.nrip++] = (Ripple){ pos[o * 2], pos[o * 2 + 1], 0.0, 1.6 * w->ripples_amt };
    }
    for (int r = T.nrip - 1; r >= 0; r--) {
        T.rip[r].age += dt;
        if (T.rip[r].age > TS_RIPPLE_LIFE) {
            memmove(&T.rip[r], &T.rip[r + 1], sizeof(Ripple) * (size_t)(T.nrip - 1 - r));
            T.nrip--;
        }
    }
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
        T.head[h] = (int)i;
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

/* ---------------------------------------------------------------------------------------- the text */
/* the sim's cloudFromImageData + collectOutline for an RGBA picture of the text (alpha > 60 = inside); n slots into
 * xy / rgb. Returns n, or < 0 (no text, too large, out of memory) */
static int ts_sample(const uint8_t *rgba, int W, int H, int stride, int count, float *xy, float *rgb) {
    if (!rgba || W < 3 || H < 3 || W > 8192 || H > 2048 || (int64_t)W * H > 8000000 || stride < W * 4 || count < 1)
        return -1;
    const size_t NP = (size_t)W * (size_t)H;
    uint8_t *in = (uint8_t *)malloc(NP), *visited = (uint8_t *)calloc(NP, 1);
    int *edges = (int *)malloc(sizeof(int) * NP);
    int rc = -3;
    int *cstart = NULL, *clist = NULL, *chain_pt = NULL, *chain_at = NULL;
    double *cum = NULL, *pts = NULL;                         /* pts: the resampled points, canvas px (doubles, as the sim) */
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
    pts = (double *)malloc(sizeof(double) * 2 * (size_t)count);
    if (!cum || !alloc_n || !pts) goto out;
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
    /* to world units: 4.5 tall, at most 16 wide, centred, flat; the sim's rainbow by x */
    double sh = 4.5, sw = 4.5 * ((double)W / (double)H);
    if (sw > 16.0) {
        const double k = 16.0 / sw;
        sw *= k;
        sh *= k;
    }
    for (int k = 0; k < count; k++) {
        const double fx = pts[k * 2], fy = pts[k * 2 + 1];
        const float x = (float)((fx / W - 0.5) * sw), y = (float)((0.5 - fy / H) * sh);
        xy[k * 2] = x;
        xy[k * 2 + 1] = y;
        ts_hsl(((double)x / sw + 0.5) * 0.8 + 0.02, 0.85, 0.58, rgb + k * 3);
    }
    rc = count;
out:
    free(in); free(visited); free(edges); free(cstart); free(clist); free(chain_pt); free(chain_at); free(cum); free(alloc_n);
    free(pts);
    return rc;
}

static float g_xy[TS_MAXN * 2], g_rgb[TS_MAXN * 3];

/* the mode's text, as pixels (RGBA, its alpha): the drones fly to it. The first text places them on it (the sim's
 * mount: they rest at the text, the size knob's space then spreads them). Returns the drone count, or < 0. */
int tx_text(const uint8_t *rgba, int W, int H, int stride) {
    if (!T.ready) return -9;
    int rc = ts_sample(rgba, W, H, stride, T.n, g_xy, g_rgb);
    if (rc < 0) return rc;
    if (!T.have_text) {
        ts_create(g_xy, g_rgb, T.n, T.rng ? T.rng : 7u);
        T.have_text = 1;
        T.fresh = 1;
    } else {
        ts_retarget(g_xy, g_rgb);
    }
    return rc;
}

/* for the tests: the slots tx_text would make (no swarm needed) */
int tx_sample(const uint8_t *rgba, int W, int H, int stride, int count, float *xy, float *rgb) {
    if (!xy || !rgb || count < 1 || count > TS_MAXN) return -1;
    return ts_sample(rgba, W, H, stride, count, xy, rgb);
}

/* for the tests: a swarm resting at a cloud (createSwarm), one physics step (stepSwarm), the state */
int tx_set_cloud(const float *xy, const float *rgb, int n, uint32_t seed) {
    if (!xy || !rgb || n < 1 || n > TS_MAXN) return -1;
    ts_create(xy, rgb, n, seed);
    T.have_text = 1;
    T.ready = 1;
    return n;
}
int tx_physics(double dt, double sc, const double *prm, const double *wave) {
    if (!T.ready || !prm) return -1;
    SwParams p = { prm[0], prm[1], prm[2], prm[3], prm[4] };
    SwWave w;
    if (wave) {
        w = (SwWave){ wave[0], wave[1], wave[2], wave[3], wave[4], wave[5], wave[6] };
    }
    ts_step(dt, &p, sc, wave ? &w : NULL);
    return 0;
}
int tx_seeds(float *seed, float *quant, int n) {
    if (n < 0 || n > T.n) return -1;
    if (seed) memcpy(seed, T.seed, sizeof(float) * (size_t)n);
    if (quant) memcpy(quant, T.quant, sizeof(float) * (size_t)n);
    return T.n;
}
int tx_get(float *pos, float *vel, float *col, float *rad, int n) {
    if (n < 0 || n > T.n) return -1;
    if (pos) memcpy(pos, T.pos, sizeof(float) * 2 * (size_t)n);
    if (vel) memcpy(vel, T.vel, sizeof(float) * 2 * (size_t)n);
    if (col) memcpy(col, T.col, sizeof(float) * 3 * (size_t)n);
    if (rad) memcpy(rad, T.rad, sizeof(float) * (size_t)n);
    return T.n;
}

/* ---------------------------------------------------------------------------------------- set-up */
int fx_init(int n, int seed) {
    memset(&T, 0, sizeof T);
    n = n < 16 ? 16 : (n > TS_MAXN ? TS_MAXN : n);
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
    double tp = gl_now_ms();
    ts_step(dt, &sp, sc, &wv);
    T.phys_ms = (float)(gl_now_ms() - tp);

    /* the view: the swarm's box, fitted to the render (full width), eased as the sim's Constrain view */
    const int N = T.n;
    double minx = 1.0e30, maxx = -1.0e30, miny = 1.0e30, maxy = -1.0e30;
    for (int i = 0; i < N; i++) {
        const double x = T.pos[i * 2], y = T.pos[i * 2 + 1];
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
        const float sx = cxs + (T.pos[i * 2] - ccx) * zoom, sy = cys - (T.pos[i * 2 + 1] - ccy) * zoom;
        float cr = T.col[i * 3], cg = T.col[i * 3 + 1], cb = T.col[i * 3 + 2];
        if (turning) {                                         /* Rodrigues about (1,1,1)/sqrt(3) */
            const float dot = hk * (cr + cg + cb);
            const float kxr = hk * (cb - cg), kxg = hk * (cr - cb), kxb = hk * (cg - cr);
            const float nr = cr * hc + kxr * hs + hk * dot * (1.0f - hc);
            const float ng = cg * hc + kxg * hs + hk * dot * (1.0f - hc);
            const float nb = cb * hc + kxb * hs + hk * dot * (1.0f - hc);
            cr = nr; cg = ng; cb = nb;
        }
        const float tw = 0.9f + tw_amp * gl_sin(T.t * tw_rate + T.seed[i] * 120.0f);
        const float a = amp0 * tw;
        gl_sprite(sx, sy, r, a * fmaxf(cr, 0.0f), a * fmaxf(cg, 0.0f), a * fmaxf(cb, 0.0f));
    }
    const float bg[3] = { gl_c01(P[P_BG_R]), gl_c01(P[P_BG_G]), gl_c01(P[P_BG_B]) };
    gl_end(bloom_amt, decay, 1.0f, bg, P[P_PREFILLED] > 0.5f);
    return 0;
}

/* for tests and DEBUG: glow's stats, then the physics' ms, the zoom (px per unit), the ripples */
void fx_stats(float *out, int n) {
    float v[11];
    gl_stats(v);
    v[8] = T.phys_ms;
    v[9] = T.last_zoom;
    v[10] = (float)T.nrip;
    for (int i = 0; i < n && i < 11; i++) out[i] = v[i];
}
