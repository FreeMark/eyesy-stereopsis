/* circuit.c - the kernel of the EYESY mode "S - Circuit": a printed circuit board, generated once, with the music
 * running through it as data. Drawn with glow.h (the Swarm Visualizer's light).
 *
 * The board (made by fx_init from a seed): a CPU in the middle, memory chips and smaller ICs around it, a connector
 * on two edges, a crystal, passives and LEDs. Every chip side's pins leave as a bus - parallel traces that walk out
 * together, straight and in 45-degree bends (the corners mitred, so the traces keep their spacing), until they meet
 * something or have gone far enough, and end in a row of vias. Short free traces fill the space between. An
 * occupancy grid (half a unit a cell) keeps traces apart. Silkscreen: chip outlines, pin-1 marks, reference
 * designators, a title.
 *
 * The music: each chip listens to part of the spectrum (the CPU to the kicks); when its part rises above its running
 * level the chip sends a word down one of its buses - a pulse of light on each 1-bit of a random pattern, racing to the
 * vias, which flash when it arrives. A kick fires every one of the CPU's buses at once; Trigger fires the whole board.
 * The CPU's die glows with the bass, the crystal ticks, LEDs flicker with the treble.
 *
 * The camera looks down on the board at an angle and drifts across it; knob 1 = how far away, knob 2 = the board
 * turning (centre still). Pulses take the foreground colour (knob 4). */
#include "glow.h"
#include <stdio.h>

#define CI_VERSION 1
#define BW_U 160                   /* the board, units: x in [-80, 80], z in [-48, 48] */
#define BH_U 96
#define GR 2                       /* occupancy cells per unit */
#define GW (BW_U * GR)
#define GH (BH_U * GR)
#define MAXPTS 24000
#define MAXTR 2400
#define MAXCHIP 24
#define MAXVIA 3000
#define MAXPAD 3000
#define MAXSEG 3000                /* silkscreen / outline strokes */
#define MAXPULSE 800
#define MAXLED 24
#define MAXBUS 400
#define BUSW 16

enum {
    P_DT, P_K1, P_K2, P_K3, P_K4, P_K5,
    P_FG_R, P_FG_G, P_FG_B, P_BG_R, P_BG_G, P_BG_B,
    P_LEVEL, P_BASS, P_MID, P_TREBLE, P_BEAT, P_KICK, P_TRIG, P_PREFILLED,
    P_WAVE_HZ,
    P_COUNT
};

typedef struct { float x, z; } V2;

typedef struct {
    int p0, np;                    /* points g_pts[p0 .. p0 + np) */
    float len;
    int bus, chip;                 /* -1: none */
    float w;                       /* half-width, units */
} Trace;

typedef struct {
    float x, z, w, h;              /* centre, size (x, z) */
    int kind;                      /* 0 CPU, 1 memory, 2 IC, 3 connector, 4 crystal */
    int band0, band1;              /* the bands it listens to */
    int bus0, nbus;                /* its buses */
    float level, mean, refr, glow;
    char label[8];
} Chip;

typedef struct { int t0, n, chip; } Bus;
typedef struct { float x, z, r, flash; } Via;
typedef struct { float x0, z0, x1, z1, w; } Pad;       /* a pad: a short thick line */
typedef struct { float x0, z0, x1, z1, w, b; } Seg;    /* silkscreen / outline, brightness */
typedef struct { int tr; float s, speed, amp; } Pulse;
typedef struct { float x, z, lvl, th; } Led;

static struct {
    int ready;
    V2 pts[MAXPTS];
    float cum[MAXPTS];             /* arc length at each point, per trace */
    int npts;
    Trace tr[MAXTR];
    int ntr;
    Chip chip[MAXCHIP];
    int nchip;
    Bus bus[MAXBUS];
    int nbus;
    Via via[MAXVIA];
    int nvia;
    Pad pad[MAXPAD];
    int npad;
    Seg seg[MAXSEG];
    int nseg;
    Pulse pulse[MAXPULSE];
    int npulse;
    Led led[MAXLED];
    int nled;
    int trace_via[MAXTR];          /* the via at each trace's end (-1 none) */
    float t, yaw, clock_ph, surge, surge_r, kick_glow;
    int word_rr;
} B;

static uint16_t g_grid[GW * GH];   /* owner of each cell: 0 free, else a bus id + 1 (or a chip's mark) */

int fx_version(void) { return CI_VERSION; }
int fx_param_count(void) { return P_COUNT; }
int fx_ready(void) { return B.ready; }
int fx_threads(int n) { return gl_set_threads(n); }
void fx_trails_reset(void) { gl_trails_reset(); }
float fx_bench(int which, int n, float r) { return gl_bench(which, n, r); }

/* ---------------------------------------------------------------------------------------- the grid */
static inline int cell_of(float x, float z, int *cx, int *cz) {
    float fx = (x + BW_U * 0.5f) * GR, fz = (z + BH_U * 0.5f) * GR;
    if (!(fx >= 0.0f && fz >= 0.0f && fx < (float)GW && fz < (float)GH)) return 0;
    *cx = (int)fx;
    *cz = (int)fz;
    return 1;
}

/* the cells within r of the segment a-b: owned by someone other than `me` (or off the board) = blocked */
static int seg_blocked(V2 a, V2 b, float r, int me, float margin) {
    float dx = b.x - a.x, dz = b.z - a.z, len = sqrtf(dx * dx + dz * dz);
    int n = (int)(len * GR * 2.0f) + 1;
    for (int i = 0; i <= n; i++) {
        float t = (float)i / (float)n, x = a.x + dx * t, z = a.z + dz * t;
        if (x < -BW_U * 0.5f + margin || x > BW_U * 0.5f - margin || z < -BH_U * 0.5f + margin || z > BH_U * 0.5f - margin)
            return 1;
        for (int oz = -1; oz <= 1; oz++)
            for (int ox = -1; ox <= 1; ox++) {
                int cx, cz;
                float sx = x + (float)ox * r, sz = z + (float)oz * r;
                if (!cell_of(sx, sz, &cx, &cz)) return 1;
                uint16_t o = g_grid[cz * GW + cx];
                if (o && o != (uint16_t)me) return 1;
            }
    }
    return 0;
}

static void seg_mark(V2 a, V2 b, float r, int me) {
    float dx = b.x - a.x, dz = b.z - a.z, len = sqrtf(dx * dx + dz * dz);
    int n = (int)(len * GR * 2.0f) + 1;
    for (int i = 0; i <= n; i++) {
        float t = (float)i / (float)n, x = a.x + dx * t, z = a.z + dz * t;
        for (int oz = -1; oz <= 1; oz++)
            for (int ox = -1; ox <= 1; ox++) {
                int cx, cz;
                if (cell_of(x + (float)ox * r, z + (float)oz * r, &cx, &cz)) g_grid[cz * GW + cx] = (uint16_t)me;
            }
    }
}

static void rect_mark(float x0, float z0, float x1, float z1, int me) {
    for (float z = z0; z <= z1; z += 0.5f / GR)
        for (float x = x0; x <= x1; x += 0.5f / GR) {
            int cx, cz;
            if (cell_of(x, z, &cx, &cz)) g_grid[cz * GW + cx] = (uint16_t)me;
        }
}

static int rect_free(float x0, float z0, float x1, float z1) {
    for (float z = z0; z <= z1; z += 0.5f / GR)
        for (float x = x0; x <= x1; x += 0.5f / GR) {
            int cx, cz;
            if (!cell_of(x, z, &cx, &cz) || g_grid[cz * GW + cx]) return 0;
        }
    return 1;
}

/* ---------------------------------------------------------------------------------------- building blocks */
static const float DX8[8] = { 1.0f, 0.70710678f, 0.0f, -0.70710678f, -1.0f, -0.70710678f, 0.0f, 0.70710678f };
static const float DZ8[8] = { 0.0f, 0.70710678f, 1.0f, 0.70710678f, 0.0f, -0.70710678f, -1.0f, -0.70710678f };

static void add_seg(float x0, float z0, float x1, float z1, float w, float b) {
    if (B.nseg >= MAXSEG) return;
    Seg *s = &B.seg[B.nseg++];
    s->x0 = x0; s->z0 = z0; s->x1 = x1; s->z1 = z1; s->w = w; s->b = b;
}

static void add_pad(float x0, float z0, float x1, float z1, float w) {
    if (B.npad >= MAXPAD) return;
    Pad *p = &B.pad[B.npad++];
    p->x0 = x0; p->z0 = z0; p->x1 = x1; p->z1 = z1; p->w = w;
}

static int add_via(float x, float z, float r) {
    if (B.nvia >= MAXVIA) return -1;
    Via *v = &B.via[B.nvia];
    v->x = x; v->z = z; v->r = r; v->flash = 0.0f;
    return B.nvia++;
}

/* a finished trace from the point list [p0, p0 + np) */
static int add_trace(int p0, int np, int bus, int chip, float w, int via_end) {
    if (B.ntr >= MAXTR || np < 2) return -1;
    Trace *t = &B.tr[B.ntr];
    t->p0 = p0; t->np = np; t->bus = bus; t->chip = chip; t->w = w;
    float L = 0.0f;
    B.cum[p0] = 0.0f;
    for (int i = 1; i < np; i++) {
        float dx = B.pts[p0 + i].x - B.pts[p0 + i - 1].x, dz = B.pts[p0 + i].z - B.pts[p0 + i - 1].z;
        L += sqrtf(dx * dx + dz * dz);
        B.cum[p0 + i] = L;
    }
    t->len = L;
    B.trace_via[B.ntr] = via_end ? add_via(B.pts[p0 + np - 1].x, B.pts[p0 + np - 1].z, 0.42f) : -1;
    return B.ntr++;
}

/* ---------------------------------------------------------------------------------------- routing */
/* a bus: n traces from their starts (pad ends, in a row across dir, pitch apart), walking out together. Returns the
 * number of traces made */
static int route_bus(const V2 *start, int n, int dir, float max_len, int chip, int owner, int via_end, float w) {
    if (n < 1 || n > BUSW || B.npts + n * 40 > MAXPTS || B.ntr + n > MAXTR) return 0;
    V2 last[BUSW];
    float off[BUSW];
    int np[BUSW];
    V2 spine = { 0.0f, 0.0f };
    for (int k = 0; k < n; k++) { spine.x += start[k].x; spine.z += start[k].z; }
    spine.x /= (float)n;
    spine.z /= (float)n;
    const int dl = (dir + 2) & 7;                                 /* left of dir */
    /* the traces' points go in per-trace staging lists, copied out when the bus is done */
    static V2 stage[BUSW][40];
    for (int k = 0; k < n; k++) {
        last[k] = start[k];
        off[k] = (start[k].x - spine.x) * DX8[dl] + (start[k].z - spine.z) * DZ8[dl];
        stage[k][0] = start[k];
        np[k] = 1;
    }
    int d = dir, turns = 0;
    float total = 0.0f;
    float L = 1.2f + gl_rnd() * 2.5f;                               /* the escape from the pads */
    for (int step = 0; step < 36 && total < max_len; step++) {
        V2 nw[BUSW];
        int ok = 0;
        for (int tries = 0; tries < 3 && !ok; tries++) {
            ok = 1;
            for (int k = 0; k < n && ok; k++) {
                nw[k].x = last[k].x + DX8[d] * L;
                nw[k].z = last[k].z + DZ8[d] * L;
                if (seg_blocked(last[k], nw[k], 0.3f, owner, 2.5f)) ok = 0;
            }
            if (!ok) L *= 0.5f;
            if (L < 0.8f) break;
        }
        if (!ok) break;
        for (int k = 0; k < n; k++) {
            seg_mark(last[k], nw[k], 0.3f, owner);
            if (np[k] < 40) stage[k][np[k]++] = nw[k];
            last[k] = nw[k];
        }
        spine.x += DX8[d] * L;
        spine.z += DZ8[d] * L;
        total += L;
        /* the next direction: a bus on an axis bends 45 degrees now and then; on a diagonal it soon bends back */
        int axis = (d & 1) == 0, nd = d;
        float u = gl_rnd();
        if (axis ? u < 0.45f : u < 0.8f) {
            int sgn = axis ? (gl_rnd() < 0.5f ? 1 : -1) : (((d - dir) & 7) == 1 ? -1 : (((d - dir) & 7) == 7 ? 1 : (gl_rnd() < 0.5f ? 1 : -1)));
            nd = (d + sgn + 8) & 7;
            if (((nd - dir + 8) & 7) == 4) nd = d;                    /* never back toward the chip */
        }
        if (nd != d && turns < 12) {
            /* mitre the corner: each trace's last point slides along d by off x tan(22.5), inward on the inside of the
             * turn, so the traces keep their spacing round it (the slid bit is checked, then marked) */
            float sgn = ((nd - d) & 7) == 1 ? 1.0f : -1.0f;             /* 1 = turning left (counter-clockwise) */
            V2 adj[BUSW];
            int fine = 1;
            for (int k = 0; k < n && fine; k++) {
                float sh = -sgn * off[k] * 0.41421356f;
                adj[k].x = last[k].x + DX8[d] * sh;
                adj[k].z = last[k].z + DZ8[d] * sh;
                if (sh > 0.0f && seg_blocked(last[k], adj[k], 0.3f, owner, 2.5f)) fine = 0;
            }
            if (fine) {
                for (int k = 0; k < n; k++) {
                    if (np[k] >= 2) {
                        seg_mark(last[k], adj[k], 0.3f, owner);
                        stage[k][np[k] - 1] = adj[k];
                    }
                    last[k] = adj[k];
                }
                d = nd;
                turns++;
            }
        }
        L = (d & 1) ? 1.5f + gl_rnd() * 5.0f : 2.5f + gl_rnd() * 9.0f;
    }
    int made = 0;
    for (int k = 0; k < n; k++) {
        if (np[k] < 2 || B.npts + np[k] > MAXPTS) continue;
        int p0 = B.npts;
        for (int i = 0; i < np[k]; i++) B.pts[B.npts++] = stage[k][i];
        if (add_trace(p0, np[k], B.nbus, chip, w, via_end) >= 0) made++;
        else B.npts = p0;
    }
    return made;
}

/* a chip: its body marked, its pins' pads, a bus from each side that has pins (split into buses of <= per) */
static void chip_pins(int ci, int sides_mask, int pins_per_side, float pitch, int per, float reach) {
    Chip *c = &B.chip[ci];
    c->bus0 = B.nbus;
    c->nbus = 0;
    for (int side = 0; side < 4; side++) {
        if (!(sides_mask & (1 << side))) continue;
        int dir = side * 2;                                       /* 0 +x, 1 +z, 2 -x, 3 -z */
        float ox = DX8[dir], oz = DZ8[dir];
        float half_along = (side & 1) ? c->w * 0.5f : c->h * 0.5f;
        float edge = (side & 1) ? c->h * 0.5f : c->w * 0.5f;
        int npin = pins_per_side;
        float span = (float)(npin - 1) * pitch;
        if (span > 2.0f * half_along - 1.0f) {
            npin = (int)((2.0f * half_along - 1.0f) / pitch) + 1;
            span = (float)(npin - 1) * pitch;
        }
        V2 st[64];
        for (int i = 0; i < npin && i < 64; i++) {
            float a = -span * 0.5f + (float)i * pitch;
            float px = c->x + ox * edge + (side & 1 ? a : 0.0f), pz = c->z + oz * edge + (side & 1 ? 0.0f : a);
            add_pad(px, pz, px + ox * 1.0f, pz + oz * 1.0f, 0.22f);
            st[i].x = px + ox * 1.0f;
            st[i].z = pz + oz * 1.0f;
        }
        for (int g = 0; g < npin; g += per) {
            int n = npin - g < per ? npin - g : per;
            if (B.nbus >= MAXBUS) break;
            int owner = B.nbus + 1;
            int made = route_bus(&st[g], n, dir, reach * (0.6f + 0.8f * gl_rnd()), ci, owner, 1, 0.13f);
            if (made) {
                Bus *b = &B.bus[B.nbus];
                b->t0 = B.ntr - made;
                b->n = made;
                b->chip = ci;
                B.nbus++;
                c->nbus++;
            }
        }
    }
}

static int place_chip(float w, float h, int kind, const char *label, float margin) {
    if (B.nchip >= MAXCHIP) return -1;
    for (int tries = 0; tries < 400; tries++) {
        float x = (gl_rnd() - 0.5f) * (BW_U - w - 24.0f), z = (gl_rnd() - 0.5f) * (BH_U - h - 20.0f);
        if (!rect_free(x - w * 0.5f - margin, z - h * 0.5f - margin, x + w * 0.5f + margin, z + h * 0.5f + margin)) continue;
        Chip *c = &B.chip[B.nchip];
        memset(c, 0, sizeof *c);
        c->x = x; c->z = z; c->w = w; c->h = h; c->kind = kind;
        for (int k = 0; k < (int)sizeof c->label - 1 && label[k]; k++) c->label[k] = label[k];
        rect_mark(x - w * 0.5f - 0.3f, z - h * 0.5f - 0.3f, x + w * 0.5f + 0.3f, z + h * 0.5f + 0.3f, 60000);
        return B.nchip++;
    }
    return -1;
}

/* ---------------------------------------------------------------------------------------- silkscreen text */
/* text at (x, z), glyph height h units, reading along +x (z up the glyph) */
static void add_text(const char *s, float x, float z, float h, float b) {
    float k = h / 6.0f, adv = 5.2f * k;
    for (int i = 0; s[i]; i++, x += adv) {
        const unsigned char *g = gl_glyph(s[i]);
        if (!g) continue;
        for (int j = 0; g[j] != 0xFF; j += 4)
            add_seg(x + g[j] * k, z - g[j + 1] * k, x + g[j + 2] * k, z - g[j + 3] * k, 0.07f * h / 1.2f, b);
    }
}

static void chip_silk(const Chip *c) {
    float x0 = c->x - c->w * 0.5f, x1 = c->x + c->w * 0.5f, z0 = c->z - c->h * 0.5f, z1 = c->z + c->h * 0.5f;
    add_seg(x0, z0, x1, z0, 0.07f, 1.0f);
    add_seg(x1, z0, x1, z1, 0.07f, 1.0f);
    add_seg(x1, z1, x0, z1, 0.07f, 1.0f);
    add_seg(x0, z1, x0, z0, 0.07f, 1.0f);
    float m = fminf(c->w, c->h) * 0.12f;                       /* pin 1: a notch in a corner */
    add_seg(x0 + m, z0 + m * 2.0f, x0 + m * 2.0f, z0 + m, 0.07f, 1.0f);
    float h = fminf(1.6f, fminf(c->w, c->h) * 0.3f);
    int len = (int)strlen(c->label);
    add_text(c->label, c->x - (float)len * 5.2f * h / 12.0f, c->z + h * 0.5f, h, 0.8f);
}

/* ---------------------------------------------------------------------------------------- the board */
int fx_init(int n, int seed) {
    (void)n;
    memset(&B, 0, sizeof B);
    memset(g_grid, 0, sizeof g_grid);
    gl_rng_state = 0x9E3779B9u ^ ((uint32_t)seed * 2654435761u);
    if (!gl_rng_state) gl_rng_state = 1u;
    /* the CPU, in the middle, 16 pins a side, a bus of 8 per half side */
    Chip *cpu = &B.chip[B.nchip++];
    cpu->x = 0.0f; cpu->z = 0.0f; cpu->w = 18.0f; cpu->h = 18.0f; cpu->kind = 0;
    snprintf(cpu->label, sizeof cpu->label, "U1");
    rect_mark(-9.3f, -9.3f, 9.3f, 9.3f, 60000);
    /* the others placed first (so the buses route around every body), then all routed */
    int mem[3], nmem = 0;
    for (int i = 0; i < 3; i++) {
        char lab[8] = { 'U', (char)('2' + i), 0 };
        int ci = place_chip(6.0f, 13.0f, 1, lab, 7.0f);
        if (ci >= 0) mem[nmem++] = ci;
    }
    int ics[MAXCHIP], nic = 0;
    for (int i = 0; i < 12 && nic < MAXCHIP; i++) {
        char lab[8] = { 'U', (char)('0' + (5 + i) / 10), (char)('0' + (5 + i) % 10), 0 };
        if (5 + i < 10) { lab[1] = (char)('0' + 5 + i); lab[2] = 0; }
        float s = 4.0f + 3.0f * gl_rnd();
        int ci = place_chip(s, i % 3 == 0 ? s * 1.5f : s, 2, lab, 6.0f);
        if (ci >= 0) ics[nic++] = ci;
    }
    /* connectors: a header down the left edge, one along the bottom */
    int jl = B.nchip;
    if (B.nchip < MAXCHIP) {
        Chip *c = &B.chip[B.nchip++];
        c->x = -BW_U * 0.5f + 5.0f; c->z = 4.0f; c->w = 3.0f; c->h = 24.0f; c->kind = 3;
        snprintf(c->label, sizeof c->label, "J1");
        rect_mark(c->x - 1.8f, c->z - 12.3f, c->x + 1.8f, c->z + 12.3f, 60000);
    }
    int jb = B.nchip;
    if (B.nchip < MAXCHIP) {
        Chip *c = &B.chip[B.nchip++];
        c->x = 12.0f; c->z = -BH_U * 0.5f + 5.0f; c->w = 30.0f; c->h = 3.0f; c->kind = 3;
        snprintf(c->label, sizeof c->label, "J2");
        rect_mark(c->x - 15.3f, c->z - 1.8f, c->x + 15.3f, c->z + 1.8f, 60000);
    }
    int xtal = place_chip(5.0f, 2.2f, 4, "Y1", 3.0f);
    /* route: the CPU first (the widest reach), then memory, ICs, connectors */
    chip_pins(0, 15, 16, 1.0f, 8, 38.0f);
    for (int i = 0; i < nmem; i++) chip_pins(mem[i], 1 | 4, 12, 1.0f, 6, 30.0f);
    for (int i = 0; i < nic; i++) {
        Chip *c = &B.chip[ics[i]];
        int pins = (int)(fminf(c->w, c->h) - 1.0f);
        chip_pins(ics[i], c->h > c->w * 1.2f ? (1 | 4) : 15, pins < 2 ? 2 : pins, 1.0f, 8, 26.0f);
    }
    chip_pins(jl, 1, 12, 2.0f, 12, 28.0f);
    chip_pins(jb, 2, 14, 2.0f, 7, 24.0f);
    if (xtal >= 0) chip_pins(xtal, 1 | 4, 1, 1.0f, 1, 10.0f);
    /* free traces between vias, filling the gaps */
    for (int i = 0; i < 700 && B.nbus < MAXBUS - 1; i++) {
        V2 s;
        s.x = (gl_rnd() - 0.5f) * (BW_U - 12.0f);
        s.z = (gl_rnd() - 0.5f) * (BH_U - 12.0f);
        if (!rect_free(s.x - 0.8f, s.z - 0.8f, s.x + 0.8f, s.z + 0.8f)) continue;
        int dir = (int)(gl_rnd() * 8.0f) & 7;
        int nb = 1 + (gl_rnd() < 0.35f ? (int)(gl_rnd() * 4.0f) + 1 : 0);
        V2 st[6];
        int dl = (dir + 2) & 7;
        for (int k = 0; k < nb; k++) {
            st[k].x = s.x + DX8[dl] * (float)k;
            st[k].z = s.z + DZ8[dl] * (float)k;
        }
        int ok = 1;
        for (int k = 0; k < nb && ok; k++) ok = rect_free(st[k].x - 0.5f, st[k].z - 0.5f, st[k].x + 0.5f, st[k].z + 0.5f);
        if (!ok) continue;
        int t0 = B.ntr;
        int made = route_bus(st, nb, dir, 6.0f + gl_rnd() * 20.0f, -1, B.nbus + 1, 1, 0.13f);
        if (made) {
            for (int k = t0; k < B.ntr; k++) add_via(B.pts[B.tr[k].p0].x, B.pts[B.tr[k].p0].z, 0.42f);
            Bus *b = &B.bus[B.nbus];
            b->t0 = t0; b->n = made; b->chip = -1;
            B.nbus++;
        }
    }
    /* passives (two pads, an outline) and LEDs in the space left */
    int npass = 0;
    for (int i = 0; i < 900 && npass < 110; i++) {
        float x = (gl_rnd() - 0.5f) * (BW_U - 10.0f), z = (gl_rnd() - 0.5f) * (BH_U - 10.0f);
        int vert = gl_rnd() < 0.5f;
        float hx = vert ? 0.6f : 1.2f, hz = vert ? 1.2f : 0.6f;
        if (!rect_free(x - hx - 0.4f, z - hz - 0.4f, x + hx + 0.4f, z + hz + 0.4f)) continue;
        rect_mark(x - hx - 0.2f, z - hz - 0.2f, x + hx + 0.2f, z + hz + 0.2f, 60001);
        if (vert) {
            add_pad(x, z - 1.0f, x, z - 0.45f, 0.42f);
            add_pad(x, z + 0.45f, x, z + 1.0f, 0.42f);
        } else {
            add_pad(x - 1.0f, z, x - 0.45f, z, 0.42f);
            add_pad(x + 0.45f, z, x + 1.0f, z, 0.42f);
        }
        add_seg(x - hx, z - hz, x + hx, z - hz, 0.05f, 0.6f);
        add_seg(x + hx, z - hz, x + hx, z + hz, 0.05f, 0.6f);
        add_seg(x + hx, z + hz, x - hx, z + hz, 0.05f, 0.6f);
        add_seg(x - hx, z + hz, x - hx, z - hz, 0.05f, 0.6f);
        if (B.nled < MAXLED && npass % 5 == 2) {
            Led *l = &B.led[B.nled++];
            l->x = x; l->z = z; l->th = 0.15f + 0.5f * gl_rnd();
        }
        npass++;
    }
    /* silkscreen: chips, the board's outline and mounting holes, a title */
    for (int i = 0; i < B.nchip; i++) chip_silk(&B.chip[i]);
    const float ex = BW_U * 0.5f - 0.5f, ez = BH_U * 0.5f - 0.5f;
    add_seg(-ex, -ez, ex, -ez, 0.12f, 0.9f);
    add_seg(ex, -ez, ex, ez, 0.12f, 0.9f);
    add_seg(ex, ez, -ex, ez, 0.12f, 0.9f);
    add_seg(-ex, ez, -ex, -ez, 0.12f, 0.9f);
    for (int i = 0; i < 4; i++) {
        float hx = (i & 1) ? ex - 3.5f : -ex + 3.5f, hz = (i & 2) ? ez - 3.5f : -ez + 3.5f;
        add_via(hx, hz, 1.6f);
    }
    add_text("STEREOPSIS", BW_U * 0.5f - 44.0f, -BH_U * 0.5f + 9.0f, 2.4f, 1.0f);
    add_text("REV 0.6", BW_U * 0.5f - 20.0f, -BH_U * 0.5f + 5.0f, 1.4f, 0.8f);
    /* who listens to what: the CPU to the kicks (bands 0-3), memory to the low mids, the ICs spread over the rest */
    B.chip[0].band0 = 0; B.chip[0].band1 = 3;
    for (int i = 1; i < B.nchip; i++) {
        int a = 4 + (int)((float)(i - 1) / (float)(B.nchip > 1 ? B.nchip - 1 : 1) * 26.0f);
        B.chip[i].band0 = a;
        B.chip[i].band1 = a + 2;
    }
    B.yaw = 0.35f;
    B.ready = 1;
    return 0;
}

/* ---------------------------------------------------------------------------------------- the music -> pulses */
static void fire(int tr, float amp, float speed) {
    if (tr < 0 || tr >= B.ntr || B.npulse >= MAXPULSE) return;
    Pulse *p = &B.pulse[B.npulse++];
    p->tr = tr; p->s = 0.0f; p->amp = amp; p->speed = speed;
}

/* a word down a bus: a pulse on each 1-bit (all bits if full) */
static void send_word(int bi, float amp, int full) {
    if (bi < 0 || bi >= B.nbus) return;
    const Bus *b = &B.bus[bi];
    uint32_t bits = gl_hash((uint32_t)(B.word_rr++) * 2654435761u + (uint32_t)bi);
    if (!full && (bits & 0xFFFFu) == 0) bits |= 1u;
    float speed = 26.0f + 14.0f * gl_rnd();
    for (int k = 0; k < b->n; k++)
        if (full || ((bits >> k) & 1u)) fire(b->t0 + k, amp, speed);
}

static void music(float dt, const float *bands, int nbands, float bass, float treble, int kick, int trig) {
    for (int i = 0; i < B.nchip; i++) {
        Chip *c = &B.chip[i];
        float e = 0.0f;
        int nb = 0;
        for (int k = c->band0; k <= c->band1 && k < nbands; k++, nb++) e += gl_c01(bands[k]);
        e = nb ? e / (float)nb : 0.0f;
        c->mean += (e - c->mean) * fminf(1.0f, dt * 1.5f);
        c->refr -= dt;
        c->level = e;
        if (i > 0 && c->nbus > 0 && c->refr <= 0.0f && e > c->mean * 1.18f + 0.04f) {
            send_word(c->bus0 + (int)(gl_rnd() * (float)c->nbus) % c->nbus, 0.5f + 1.2f * (e - c->mean), 0);
            c->refr = 0.07f + 0.12f * gl_rnd();
            c->glow = 1.0f;
        }
        c->glow = fmaxf(0.0f, c->glow - dt * 4.0f);
    }
    if (kick) {                                              /* the CPU fires every bus at once */
        for (int k = 0; k < B.chip[0].nbus; k++) send_word(B.chip[0].bus0 + k, 1.3f, 1);
        B.chip[0].glow = 1.0f;
        B.kick_glow = 1.0f;
    }
    if (trig) {                                              /* a power surge: a ring of light out over the copper */
        B.surge = 1.0f;
        B.surge_r = 0.0f;
        for (int k = 0; k < B.chip[0].nbus; k++) send_word(B.chip[0].bus0 + k, 1.6f, 1);
    }
    B.kick_glow = fmaxf(0.0f, B.kick_glow - dt * 3.0f);
    B.surge = fmaxf(0.0f, B.surge - dt * 0.5f);
    B.surge_r += dt * 70.0f;
    B.chip[0].level = fmaxf(B.chip[0].level, bass);
    /* the pulses run; arriving ones flash their via */
    int w = 0;
    for (int i = 0; i < B.npulse; i++) {
        Pulse *p = &B.pulse[i];
        p->s += p->speed * dt;
        const Trace *t = &B.tr[p->tr];
        if (p->s - 7.0f > t->len) {                          /* the tail has gone in too */
            continue;
        }
        if (p->s >= t->len && p->s - p->speed * dt < t->len) {
            int v = B.trace_via[p->tr];
            if (v >= 0) B.via[v].flash = fminf(2.0f, B.via[v].flash + p->amp);
        }
        B.pulse[w++] = *p;
    }
    B.npulse = w;
    for (int i = 0; i < B.nvia; i++) B.via[i].flash = fmaxf(0.0f, B.via[i].flash - dt * 3.0f);
    for (int i = 0; i < B.nled; i++) {                       /* LEDs: on when the treble passes each one's level */
        Led *l = &B.led[i];
        float on = treble > l->th ? 1.0f : 0.0f;
        l->lvl += (on - l->lvl) * fminf(1.0f, dt * (on > l->lvl ? 30.0f : 6.0f));
    }
}

/* ---------------------------------------------------------------------------------------- drawing */
static GCam g_cam;

static void line_uv(float x0, float z0, float x1, float z1, float w, const float c0[3], const float c1[3]) {
    gl_line3(&g_cam, x0, 0.0f, z0, x1, 0.0f, z1, w, c0, c1);
}

/* a pulse: its tail over the trace (brightest at the head, fading over 7 units), and the head */
static void draw_pulse(const Pulse *p, const float col[3]) {
    const Trace *t = &B.tr[p->tr];
    const float TAIL = 7.0f;
    float s1 = fminf(p->s, t->len), s0 = fmaxf(p->s - TAIL, 0.0f);
    if (s1 <= s0) return;
    for (int i = 0; i + 1 < t->np; i++) {
        int a = t->p0 + i, b = a + 1;
        float ca = B.cum[a], cb = B.cum[b];
        if (cb <= s0 || ca >= s1) continue;
        float u0 = fmaxf(ca, s0), u1 = fminf(cb, s1), len = cb - ca;
        if (len <= 1e-4f) continue;
        float f0 = (u0 - ca) / len, f1 = (u1 - ca) / len;
        float x0 = B.pts[a].x + (B.pts[b].x - B.pts[a].x) * f0, z0 = B.pts[a].z + (B.pts[b].z - B.pts[a].z) * f0;
        float x1 = B.pts[a].x + (B.pts[b].x - B.pts[a].x) * f1, z1 = B.pts[a].z + (B.pts[b].z - B.pts[a].z) * f1;
        float b0 = 1.0f - (p->s - u0) / TAIL, b1 = 1.0f - (p->s - u1) / TAIL;
        b0 = b0 * b0 * p->amp;
        b1 = b1 * b1 * p->amp;
        float c0[3] = { col[0] * b0, col[1] * b0, col[2] * b0 }, c1[3] = { col[0] * b1, col[1] * b1, col[2] * b1 };
        line_uv(x0, z0, x1, z1, t->w * 1.6f, c0, c1);
    }
    if (p->s <= t->len) {                                    /* the head */
        int k = 0;
        while (k + 2 < t->np && B.cum[t->p0 + k + 1] < p->s) k++;
        int a = t->p0 + k, b = a + 1;
        float len = B.cum[b] - B.cum[a], f = len > 1e-4f ? (p->s - B.cum[a]) / len : 0.0f;
        float x = B.pts[a].x + (B.pts[b].x - B.pts[a].x) * f, z = B.pts[a].z + (B.pts[b].z - B.pts[a].z) * f;
        float h = p->amp;
        gl_sprite3(&g_cam, x, 0.0f, z, 0.55f, col[0] * h + 0.3f * h, col[1] * h + 0.3f * h, col[2] * h + 0.3f * h);
    }
}

int fx_frame(uint8_t *dst, int dW, int dH, int dpitch, int down, int sh_r, int sh_g, int sh_b, float *acc, int W, int H,
             float *bloom, int BW, int BH, const float *fft, int nfft, float fft_hz, const float *wave, int nwave,
             const float *bands, int nbands, const float *P) {
    (void)fft_hz;
    if (!B.ready) return -9;
    if (!P) return -1;
    if (nfft < 0 || nfft > 65536 || nwave < 0 || nwave > 65536 || nbands < 0 || nbands > 256 || (nfft > 0 && !fft) ||
        (nwave > 0 && !wave) || (nbands > 0 && !bands))
        return -3;
    const float k3 = gl_c01(P[P_K3]);
    int rc = gl_begin(dst, dW, dH, dpitch, down, sh_r, sh_g, sh_b, acc, W, H, bloom, BW, BH, k3 > 0.0f);
    if (rc) return rc;
    const float dt = gl_clampf(P[P_DT], 0.0f, 0.1f);
    B.t += dt;
    if (B.t > 1.0e5f) B.t = 0.0f;
    /* without the analyser's bands: the level stands in for every band (the chips still talk, all together) */
    float lv_bands[32];
    const float level = gl_c01(P[P_LEVEL]);
    if (nbands < 8 || !bands) {
        for (int i = 0; i < 32; i++) lv_bands[i] = level * (0.7f + 0.3f * gl_sin(B.t * (1.0f + 0.37f * (float)i)));
        bands = lv_bands;
        nbands = 32;
    }
    music(dt, bands, nbands, gl_c01(P[P_BASS]), gl_c01(P[P_TREBLE]), P[P_KICK] > 0.5f, P[P_TRIG] > 0.5f);

    /* the camera: looking down at the board, drifting over it; knob 1 = distance, knob 2 = the board turning */
    float k1 = gl_c01(P[P_K1]), k2 = (gl_c01(P[P_K2]) - 0.5f) * 2.0f;
    k2 = fabsf(k2) < 0.08f ? 0.0f : copysignf((fabsf(k2) - 0.08f) / 0.92f, k2);
    B.yaw += k2 * 0.35f * dt;
    if (B.yaw > 1.0e4f || B.yaw < -1.0e4f) B.yaw = 0.0f;
    const float dist = 95.0f * powf(0.15f, k1);                  /* 95 (the whole board) .. 14 (a chip) */
    const float drift = fminf(1.0f, (95.0f - dist) / 50.0f);     /* far out, the board sits in the middle */
    const float tx = drift * 42.0f * gl_sin(B.t * 0.019f), tz = drift * 24.0f * gl_sin(B.t * 0.031f + 1.0f);
    gl_camera(&g_cam, B.yaw, 0.98f + 0.08f * gl_sin(B.t * 0.05f), dist, 0.9f, tx, 0.0f, tz);

    float fg[3] = { gl_c01(P[P_FG_R]), gl_c01(P[P_FG_G]), gl_c01(P[P_FG_B]) };
    if (fg[0] + fg[1] + fg[2] < 0.15f) fg[0] = fg[1] = fg[2] = 0.6f;   /* a black foreground would hide the data */
    const float lift = 1.0f;
    /* the board: copper traces, pads, vias, silkscreen - dim, so the data shows; a surge's ring lights the copper it
     * passes (in the foreground colour) */
    const float cu[3] = { 0.95f * 0.24f, 0.55f * 0.24f, 0.22f * 0.24f };
    for (int i = 0; i < B.ntr; i++) {
        const Trace *t = &B.tr[i];
        for (int j = 0; j + 1 < t->np; j++) {
            const V2 *a = &B.pts[t->p0 + j], *b = a + 1;
            float ca[3] = { cu[0], cu[1], cu[2] }, cb[3] = { cu[0], cu[1], cu[2] };
            if (B.surge > 0.01f) {
                float da = sqrtf(a->x * a->x + a->z * a->z) - B.surge_r, db = sqrtf(b->x * b->x + b->z * b->z) - B.surge_r;
                float wa = B.surge * 2.2f / (1.0f + da * da * 0.04f), wb = B.surge * 2.2f / (1.0f + db * db * 0.04f);
                for (int c = 0; c < 3; c++) { ca[c] += fg[c] * wa; cb[c] += fg[c] * wb; }
            }
            line_uv(a->x, a->z, b->x, b->z, t->w, ca, cb);
        }
    }
    const float au[3] = { 1.0f * 0.42f * lift, 0.78f * 0.42f * lift, 0.40f * 0.42f * lift };
    for (int i = 0; i < B.npad; i++) {
        const Pad *p = &B.pad[i];
        line_uv(p->x0, p->z0, p->x1, p->z1, p->w, au, au);
    }
    for (int i = 0; i < B.nvia; i++) {
        const Via *v = &B.via[i];
        float f = v->flash;
        gl_sprite3(&g_cam, v->x, 0.0f, v->z, v->r, au[0] + fg[0] * f, au[1] + fg[1] * f, au[2] + fg[2] * f);
    }
    const float silk[3] = { 0.80f * 0.24f, 0.85f * 0.24f, 0.90f * 0.24f };
    for (int i = 0; i < B.nseg; i++) {
        const Seg *s = &B.seg[i];
        float c[3] = { silk[0] * s->b, silk[1] * s->b, silk[2] * s->b };
        line_uv(s->x0, s->z0, s->x1, s->z1, s->w, c, c);
    }
    /* chips: the die under each glows with its part of the music */
    for (int i = 0; i < B.nchip; i++) {
        const Chip *c = &B.chip[i];
        if (c->kind >= 3) continue;
        float g = 0.25f * c->level + 0.9f * c->glow + (i == 0 ? 0.8f * B.kick_glow : 0.0f);
        float r = fminf(c->w, c->h) * (i == 0 ? 0.42f : 0.36f);
        gl_sprite3(&g_cam, c->x, 0.0f, c->z, r, fg[0] * g, fg[1] * g, fg[2] * g);
    }
    /* the crystal ticks (at the tempo of the kicks, roughly: a steady clock otherwise) */
    B.clock_ph += dt * 4.0f;
    for (int i = 0; i < B.nchip; i++)
        if (B.chip[i].kind == 4) {
            float tick = gl_fract(B.clock_ph) < 0.15f ? 0.8f : 0.1f;
            gl_sprite3(&g_cam, B.chip[i].x, 0.0f, B.chip[i].z, 0.9f, tick, tick, tick * 0.9f);
        }
    for (int i = 0; i < B.nled; i++) {                       /* LEDs: the foreground turned a third of the way round */
        const Led *l = &B.led[i];
        float b = 0.08f + 1.8f * l->lvl;
        gl_sprite3(&g_cam, l->x, 0.0f, l->z, 0.7f, fg[2] * b + 0.05f * b, fg[0] * b + 0.05f * b, fg[1] * b + 0.05f * b);
    }
    for (int i = 0; i < B.npulse; i++) draw_pulse(&B.pulse[i], fg);
    /* knob 3: bloom up to the middle, a dead band, then trails from 0.6 */
    const float bloom_amt = k3 < 0.5f ? k3 * 2.0f * 1.6f : 1.6f;
    const float decay = k3 >= 0.6f ? 0.35f + (k3 - 0.6f) * (0.55f / 0.4f) : 0.0f;
    const float bg[3] = { gl_c01(P[P_BG_R]), gl_c01(P[P_BG_G]), gl_c01(P[P_BG_B]) };
    gl_end(bloom_amt, decay, 1.0f, bg, P[P_PREFILLED] > 0.5f);
    return 0;
}

/* for tests and DEBUG: glow's stats, then traces, active pulses, vias */
void fx_stats(float *out, int n) {
    float v[11];
    gl_stats(v);
    v[8] = (float)B.ntr;
    v[9] = (float)B.npulse;
    v[10] = (float)B.nvia;
    for (int i = 0; i < n && i < 11; i++) out[i] = v[i];
}
