/* kms.c - stereopsis's display layer.
 * Started as a copy of "Z - Engine Lab"/turbo.c v2 (Phase 1, proven on the device); v3 adds tb_open +
 * tb_modeset so stereopsis owns the display itself: no SDL video, no GL, no texture uploads.
 * The history below is turbo.c's.
 *
 * SDL 2.26's KMSDRM backend presents pygame's software surface by uploading it as a GL texture through
 * Mesa vc4 (CPU tiling): ~17 ms per 1280x720 frame on the CM3 (measured). This file presents the same
 * surface the direct way: one memcpy into a linear "dumb" scanout buffer, then a KMS page flip onto the
 * CRTC that SDL already configured.
 *
 * It borrows SDL's DRM file descriptor (the DRM master) inside the engine process and NEVER reads DRM
 * events from it: SDL's last page-flip event stays queued for SDL, so SDL can take over presenting again
 * at any time (tb_handback). Flip completion is tracked with vblank counters (DRM_IOCTL_WAIT_VBLANK).
 *
 * The kernel refuses a legacy page flip that changes the pixel format, and SDL creates its framebuffers
 * with legacy AddFB(depth 24, bpp 32) = XRGB8888, so ours are XRGB8888 too; tb_init checks SDL's FB first.
 * Every value the kernel returns is range-checked; every failure returns a negative errno-style code and
 * leaves the current scanout alone.
 *
 * v2 adds the hardware render-scale path (tb_plane_*): a smaller frame goes into its own buffers on a
 * free overlay plane, and the display compositor (HVS) scales it to the destination rectangle while it
 * scans out, at no CPU cost. The legacy SETPLANE ioctl blocks until the update is on screen, so a
 * worker thread does the commit (FIFO depth 1: a present waits only if a frame is already queued).
 *
 * v4 (stereopsis 0.2) adds the compositor (tb_comp_*, at the end): one worker thread, atomic commits,
 * the mode layer scaled by the display hardware from any render size, an ARGB OSD layer on top, and the
 * frame copy + clear done off the engine's thread. tb_present/tb_plane_* remain as the fallback path.
 * v5 adds the preview (tb_preview_*): a small JPEG of the latest frame for the browser stream, made on
 * the compositor's and an encoder thread's cores, only while someone watches. Link with -ljpeg.
 */
#define _GNU_SOURCE
#include <errno.h>
#include <fcntl.h>
#include <poll.h>
#include <pthread.h>
#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <time.h>
#include <unistd.h>
#include <drm/drm.h>
#include <drm/drm_mode.h>

#define TB_VERSION 7
#define TB_MAXN 16                 /* connectors / encoders / crtcs / planes looked at */
#define TB_MAXMODES 64
#define TB_NBUF_MAX 3
#define TB_XRGB8888 0x34325258u    /* fourcc 'XR24' */

typedef struct {
    uint32_t handle, fb_id, pitch;
    uint64_t size;
    uint8_t *map;
} tb_buf;

static struct {
    int fd, ready, retired;
    uint32_t crtc_id, crtc_index, conn_id, width, height, orig_fb, refresh;
    tb_buf buf[TB_NBUF_MAX];
    int nbuf, front, pending;      /* buffer on screen (-1 = SDL's), buffer flipped but maybe not yet on screen */
    uint32_t pending_seq;          /* vblank count by which the pending flip should be done */
    uint64_t frames, flips, ebusy, vblank_waits, errors;
    double copy_ms, wait_ms, flip_ms;
    uint32_t last_seq;
    int last_err;
} T = { .fd = -1, .front = -1, .pending = -1 };

int tb_version(void) { return TB_VERSION; }

static double now_ms(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return ts.tv_sec * 1000.0 + ts.tv_nsec / 1e6;
}

static int xioctl(int fd, unsigned long req, void *arg) {
    for (int i = 0; i < 64; i++) {
        if (ioctl(fd, req, arg) == 0)
            return 0;
        if (errno != EINTR && errno != EAGAIN)
            return -errno;
    }
    return -EINTR;
}

static void sleep_us(long us) {
    struct timespec ts = { us / 1000000, (us % 1000000) * 1000 };
    nanosleep(&ts, NULL);
}

/* DRM master = allowed to modeset/flip. Same trick as libdrm's drmIsMaster: AUTH_MAGIC(0) needs master;
 * a master gets EINVAL (bad magic), anyone else EACCES. Returns 1 master, 0 not, <0 not a DRM fd. */
int tb_is_master(int fd) {
    struct drm_auth a;
    memset(&a, 0, sizeof a);
    if (fd < 0)
        return -EBADF;
    int e = xioctl(fd, DRM_IOCTL_AUTH_MAGIC, &a);
    if (e == -EACCES)
        return 0;
    if (e == -ENOTTY || e == -EBADF || e == -ENODEV)
        return e;
    return 1;
}

static int get_cap(int fd, uint64_t cap, uint64_t *val) {
    struct drm_get_cap c;
    memset(&c, 0, sizeof c);
    c.capability = cap;
    int e = xioctl(fd, DRM_IOCTL_GET_CAP, &c);
    if (!e)
        *val = c.value;
    return e;
}

/* ---- resources ------------------------------------------------------------------------------------- */
typedef struct {
    uint32_t crtcs[TB_MAXN], conns[TB_MAXN], encs[TB_MAXN];
    uint32_t n_crtcs, n_conns, n_encs;             /* as reported (may exceed TB_MAXN) */
} tb_res;

static int get_res(int fd, tb_res *r) {
    struct drm_mode_card_res cr;
    memset(r, 0, sizeof *r);
    memset(&cr, 0, sizeof cr);
    cr.crtc_id_ptr = (uint64_t)(uintptr_t)r->crtcs;
    cr.connector_id_ptr = (uint64_t)(uintptr_t)r->conns;
    cr.encoder_id_ptr = (uint64_t)(uintptr_t)r->encs;
    cr.count_crtcs = cr.count_connectors = cr.count_encoders = TB_MAXN;   /* kernel copies at most this many */
    int e = xioctl(fd, DRM_IOCTL_MODE_GETRESOURCES, &cr);
    if (e)
        return e;
    r->n_crtcs = cr.count_crtcs;
    r->n_conns = cr.count_connectors;
    r->n_encs = cr.count_encoders;
    return 0;
}

static uint32_t lim(uint32_t n) { return n < TB_MAXN ? n : TB_MAXN; }

/* count_modes > 0 on purpose: count_modes == 0 asks a DRM master for a forced re-probe (EDID read). */
static int get_conn(int fd, uint32_t id, struct drm_mode_get_connector *c, struct drm_mode_modeinfo *modes) {
    memset(c, 0, sizeof *c);
    memset(modes, 0, sizeof(struct drm_mode_modeinfo) * TB_MAXMODES);
    c->connector_id = id;
    c->modes_ptr = (uint64_t)(uintptr_t)modes;
    c->count_modes = TB_MAXMODES;
    return xioctl(fd, DRM_IOCTL_MODE_GETCONNECTOR, c);
}

static int get_enc(int fd, uint32_t id, struct drm_mode_get_encoder *e) {
    memset(e, 0, sizeof *e);
    e->encoder_id = id;
    return xioctl(fd, DRM_IOCTL_MODE_GETENCODER, e);
}

static int get_crtc(int fd, uint32_t id, struct drm_mode_crtc *c) {
    memset(c, 0, sizeof *c);
    c->crtc_id = id;
    return xioctl(fd, DRM_IOCTL_MODE_GETCRTC, c);
}

/* pixel format + modifier of an existing framebuffer; closes the GEM handles GETFB2 hands out */
static int fb_format(int fd, uint32_t fb_id, uint32_t *fmt, uint64_t *mod, uint32_t *w, uint32_t *h) {
    struct drm_mode_fb_cmd2 f;
    memset(&f, 0, sizeof f);
    f.fb_id = fb_id;
    int e = xioctl(fd, DRM_IOCTL_MODE_GETFB2, &f);
    if (e)
        return e;
    for (int i = 0; i < 4; i++) {
        if (!f.handles[i])
            continue;
        int dup = 0;
        for (int j = 0; j < i; j++)
            dup |= f.handles[j] == f.handles[i];
        if (!dup) {
            struct drm_gem_close gc = { .handle = f.handles[i] };
            xioctl(fd, DRM_IOCTL_GEM_CLOSE, &gc);
        }
    }
    *fmt = f.pixel_format;
    *mod = (f.flags & DRM_MODE_FB_MODIFIERS) ? f.modifier[0] : 0;
    *w = f.width;
    *h = f.height;
    return 0;
}

static const char *conn_name(uint32_t t) {
    switch (t) {
    case DRM_MODE_CONNECTOR_Composite: return "Composite";
    case DRM_MODE_CONNECTOR_HDMIA: return "HDMI-A";
    case DRM_MODE_CONNECTOR_DSI: return "DSI";
    case DRM_MODE_CONNECTOR_DPI: return "DPI";
    case DRM_MODE_CONNECTOR_WRITEBACK: return "Writeback";
    default: return "other";
    }
}

static void cat(char *out, int cap, int *len, const char *fmt, ...) {
    if (*len >= cap - 1)
        return;
    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(out + *len, (size_t)(cap - *len), fmt, ap);
    va_end(ap);
    if (n > 0)
        *len = (*len + n < cap - 1) ? *len + n : cap - 1;
}

static void fourcc(uint32_t f, char s[5]) {
    for (int i = 0; i < 4; i++) {
        char c = (char)((f >> (8 * i)) & 0xff);
        s[i] = (c >= 32 && c < 127) ? c : '?';
    }
    s[4] = 0;
}

/* ---- probe: read-only report of what the display hardware says ------------------------------------ */
int tb_probe(int fd, char *out, int cap) {
    if (!out || cap < 256)
        return -EINVAL;
    int len = 0;
    out[0] = 0;
    cat(out, cap, &len, "turbo.c v%d | fd %d | master %d\n", TB_VERSION, fd, tb_is_master(fd));
    static const struct { uint64_t id; const char *name; } caps[] = {
        { DRM_CAP_DUMB_BUFFER, "dumb_buffer" }, { DRM_CAP_VBLANK_HIGH_CRTC, "vblank_high_crtc" },
        { DRM_CAP_DUMB_PREFERRED_DEPTH, "dumb_depth" }, { DRM_CAP_DUMB_PREFER_SHADOW, "dumb_prefer_shadow" },
        { DRM_CAP_PRIME, "prime" }, { DRM_CAP_TIMESTAMP_MONOTONIC, "ts_monotonic" },
        { DRM_CAP_ASYNC_PAGE_FLIP, "async_flip" }, { DRM_CAP_ADDFB2_MODIFIERS, "addfb2_modifiers" },
        { DRM_CAP_CRTC_IN_VBLANK_EVENT, "crtc_in_vblank_event" } };
    cat(out, cap, &len, "caps:");
    for (unsigned i = 0; i < sizeof caps / sizeof caps[0]; i++) {
        uint64_t v = 0;
        int e = get_cap(fd, caps[i].id, &v);
        if (e)
            cat(out, cap, &len, " %s=err%d", caps[i].name, e);
        else
            cat(out, cap, &len, " %s=%llu", caps[i].name, (unsigned long long)v);
    }
    cat(out, cap, &len, "\n");
    tb_res r;
    int e = get_res(fd, &r);
    if (e) {
        cat(out, cap, &len, "GETRESOURCES failed: %d\n", e);
        return e;
    }
    cat(out, cap, &len, "resources: %u crtcs, %u connectors, %u encoders\n", r.n_crtcs, r.n_conns, r.n_encs);
    for (uint32_t i = 0; i < lim(r.n_crtcs); i++) {
        struct drm_mode_crtc c;
        if ((e = get_crtc(fd, r.crtcs[i], &c))) {
            cat(out, cap, &len, "crtc %u: err %d\n", r.crtcs[i], e);
            continue;
        }
        cat(out, cap, &len, "crtc %u (index %u): fb %u, mode_valid %u", c.crtc_id, i, c.fb_id, c.mode_valid);
        if (c.mode_valid)
            cat(out, cap, &len, ", %ux%u@%u '%.32s'", c.mode.hdisplay, c.mode.vdisplay, c.mode.vrefresh, c.mode.name);
        if (c.fb_id) {
            uint32_t fmt = 0, w = 0, h = 0;
            uint64_t mod = 0;
            char s[5];
            if (!fb_format(fd, c.fb_id, &fmt, &mod, &w, &h)) {
                fourcc(fmt, s);
                cat(out, cap, &len, ", fb %ux%u %s modifier 0x%llx", w, h, s, (unsigned long long)mod);
            }
        }
        cat(out, cap, &len, "\n");
    }
    for (uint32_t i = 0; i < lim(r.n_conns); i++) {
        struct drm_mode_get_connector c;
        struct drm_mode_modeinfo modes[TB_MAXMODES];
        if ((e = get_conn(fd, r.conns[i], &c, modes))) {
            cat(out, cap, &len, "connector %u: err %d\n", r.conns[i], e);
            continue;
        }
        cat(out, cap, &len, "connector %u %s-%u: connection %u, encoder %u, %u modes, %ux%u mm",
            c.connector_id, conn_name(c.connector_type), c.connector_type_id, c.connection, c.encoder_id,
            c.count_modes, c.mm_width, c.mm_height);
        /* the kernel copies the list only if it all fits; otherwise the buffer stays zeroed */
        if (c.count_modes > TB_MAXMODES)
            cat(out, cap, &len, " | (more than %d modes, not listed)", TB_MAXMODES);
        else
            for (uint32_t m = 0; m < c.count_modes && m < 3; m++)
                cat(out, cap, &len, " | %ux%u@%u%s", modes[m].hdisplay, modes[m].vdisplay, modes[m].vrefresh,
                    (modes[m].type & DRM_MODE_TYPE_PREFERRED) ? "*" : "");
        cat(out, cap, &len, "\n");
    }
    for (uint32_t i = 0; i < lim(r.n_encs); i++) {
        struct drm_mode_get_encoder en;
        if ((e = get_enc(fd, r.encs[i], &en))) {
            cat(out, cap, &len, "encoder %u: err %d\n", r.encs[i], e);
            continue;
        }
        cat(out, cap, &len, "encoder %u: type %u, crtc %u, possible_crtcs 0x%x\n", en.encoder_id, en.encoder_type,
            en.crtc_id, en.possible_crtcs);
    }
    uint32_t planes[TB_MAXN];
    struct drm_mode_get_plane_res pr;
    memset(&pr, 0, sizeof pr);
    pr.plane_id_ptr = (uint64_t)(uintptr_t)planes;
    pr.count_planes = TB_MAXN;
    if ((e = xioctl(fd, DRM_IOCTL_MODE_GETPLANERESOURCES, &pr))) {
        cat(out, cap, &len, "GETPLANERESOURCES: err %d\n", e);
    } else {
        cat(out, cap, &len, "planes (overlays only, no universal-planes cap): %u\n", pr.count_planes);
        for (uint32_t i = 0; i < lim(pr.count_planes); i++) {
            uint32_t fmts[32];
            struct drm_mode_get_plane p;
            memset(&p, 0, sizeof p);
            p.plane_id = planes[i];
            p.format_type_ptr = (uint64_t)(uintptr_t)fmts;
            p.count_format_types = 32;
            if ((e = xioctl(fd, DRM_IOCTL_MODE_GETPLANE, &p))) {
                cat(out, cap, &len, "plane %u: err %d\n", planes[i], e);
                continue;
            }
            cat(out, cap, &len, "plane %u: crtc %u, fb %u, possible_crtcs 0x%x, %u formats:", p.plane_id, p.crtc_id,
                p.fb_id, p.possible_crtcs, p.count_format_types);
            uint32_t nf = p.count_format_types < 32 ? p.count_format_types : 32;
            for (uint32_t f = 0; f < nf && f < 12; f++) {
                char s[5];
                fourcc(fmts[f], s);
                cat(out, cap, &len, " %s", s);
            }
            cat(out, cap, &len, "\n");
        }
    }
    return len;
}

/* ---- scanout buffers -------------------------------------------------------------------------------- */
#ifdef TB_TEST_FAKE
/* Test build only (test_plane_fake.c): no DRM. Buffers are malloc'd; set_plane "scans out" by sleeping
 * like a vblank wait and counts any moment the buffer it commits or shows is being written. */
#include <stdlib.h>
static int fake_copying = -1;
static volatile long fake_violations;
static uint32_t fake_fb = 1000;
#endif

static void free_buf(int fd, tb_buf *b) {
#ifdef TB_TEST_FAKE
    (void)fd;
    free(b->map);
    memset(b, 0, sizeof *b);
    return;
#endif
    if (b->map && b->map != MAP_FAILED)
        munmap(b->map, (size_t)b->size);
    if (b->fb_id) {
        uint32_t id = b->fb_id;
        xioctl(fd, DRM_IOCTL_MODE_RMFB, &id);
    }
    if (b->handle) {
        struct drm_mode_destroy_dumb d = { .handle = b->handle };
        xioctl(fd, DRM_IOCTL_MODE_DESTROY_DUMB, &d);
    }
    memset(b, 0, sizeof *b);
}

static int make_buf_d(int fd, uint32_t w, uint32_t h, uint32_t depth, tb_buf *b);

static int make_buf(int fd, uint32_t w, uint32_t h, tb_buf *b) {
    return make_buf_d(fd, w, h, 24, b);        /* legacy (32, 24) = XRGB8888, the same as SDL's framebuffers */
}

/* depth 24 = XRGB8888, depth 32 = ARGB8888 (legacy ADDFB maps bpp 32 + depth to the fourcc) */
static int make_buf_d(int fd, uint32_t w, uint32_t h, uint32_t depth, tb_buf *b) {
    memset(b, 0, sizeof *b);
#ifdef TB_TEST_FAKE
    (void)fd;
    (void)depth;
    b->pitch = w * 4;
    b->size = (uint64_t)b->pitch * h;
    b->map = calloc(1, (size_t)b->size);
    b->fb_id = fake_fb++;
    return b->map ? 0 : -ENOMEM;
#endif
    struct drm_mode_create_dumb c;
    memset(&c, 0, sizeof c);
    c.width = w;
    c.height = h;
    c.bpp = 32;
    int e = xioctl(fd, DRM_IOCTL_MODE_CREATE_DUMB, &c);
    if (e)
        return e;
    b->handle = c.handle;
    b->pitch = c.pitch;
    b->size = c.size;
    if (c.pitch < w * 4 || c.size < (uint64_t)c.pitch * h || c.size > (256ull << 20)) {
        free_buf(fd, b);
        return -EPROTO;
    }
    struct drm_mode_fb_cmd f;
    memset(&f, 0, sizeof f);
    f.width = w;
    f.height = h;
    f.pitch = c.pitch;
    f.bpp = 32;
    f.depth = depth;
    f.handle = c.handle;
    if ((e = xioctl(fd, DRM_IOCTL_MODE_ADDFB, &f))) {
        free_buf(fd, b);
        return e;
    }
    b->fb_id = f.fb_id;
    struct drm_mode_map_dumb m;
    memset(&m, 0, sizeof m);
    m.handle = c.handle;
    if ((e = xioctl(fd, DRM_IOCTL_MODE_MAP_DUMB, &m))) {
        free_buf(fd, b);
        return e;
    }
    void *p = mmap(NULL, (size_t)c.size, PROT_READ | PROT_WRITE, MAP_SHARED, fd, (off_t)m.offset);
    if (p == MAP_FAILED) {
        e = -errno;
        b->map = NULL;
        free_buf(fd, b);
        return e;
    }
    b->map = (uint8_t *)p;
    memset(b->map, 0, (size_t)c.size);
    return 0;
}

/* copy rows; exported so the unit test can hammer it under ASan */
int tb_copy_rows(uint8_t *dst, int dpitch, const uint8_t *src, int spitch, int row_bytes, int rows) {
    if (!dst || !src || row_bytes <= 0 || rows <= 0 || dpitch < row_bytes || spitch < row_bytes)
        return -EINVAL;
    if (dpitch == row_bytes && spitch == row_bytes) {
        memcpy(dst, src, (size_t)row_bytes * (size_t)rows);
        return 0;
    }
    for (int y = 0; y < rows; y++)
        memcpy(dst + (size_t)y * dpitch, src + (size_t)y * spitch, (size_t)row_bytes);
    return 0;
}

/* ---- stereopsis (v3): own the display. Open the DRM node, pick a connector + mode + CRTC, and set the
 * mode ourselves with our first scanout buffer (no SDL, no GL anywhere). ------------------------------- */
int tb_open(const char *path) {
    if (!path)
        return -EINVAL;
    int fd = open(path, O_RDWR | O_CLOEXEC);
    if (fd < 0)
        return -errno;
    int m = tb_is_master(fd);
    if (m == 0 && ioctl(fd, DRM_IOCTL_SET_MASTER, 0) == 0)
        m = tb_is_master(fd);
    if (m != 1) {
        close(fd);
        return m < 0 ? m : -EACCES;
    }
    return fd;
}

/* every mode of a connector, however many (a second call if 64 was not enough; count_modes is never 0,
 * which would ask the kernel for a forced re-probe) */
static int conn_modes(int fd, uint32_t id, struct drm_mode_get_connector *c, struct drm_mode_modeinfo **out,
                      uint32_t *encs, uint32_t nenc_cap) {
    uint32_t cap = TB_MAXMODES;
    for (int pass = 0; pass < 2; pass++) {
        struct drm_mode_modeinfo *m = calloc(cap, sizeof *m);
        if (!m)
            return -ENOMEM;
        memset(c, 0, sizeof *c);
        c->connector_id = id;
        c->modes_ptr = (uint64_t)(uintptr_t)m;
        c->count_modes = cap;
        c->encoders_ptr = (uint64_t)(uintptr_t)encs;
        c->count_encoders = nenc_cap;
        int e = xioctl(fd, DRM_IOCTL_MODE_GETCONNECTOR, c);
        if (e) {
            free(m);
            return e;
        }
        if (c->count_modes <= cap) {
            *out = m;
            return 0;
        }
        free(m);
        if (c->count_modes > 1024)
            return -E2BIG;
        cap = c->count_modes;
    }
    return -EAGAIN;
}

/* lower = better, -1 = wrong size. Refresh nearest want_hz; progressive over interlaced; preferred flag. */
static int mode_rank(const struct drm_mode_modeinfo *m, int w, int h, int hz) {
    if (m->hdisplay != (uint32_t)w || m->vdisplay != (uint32_t)h)
        return -1;
    int r = abs((int)m->vrefresh - hz) * 4;
    if (m->flags & DRM_MODE_FLAG_INTERLACE)
        r += 1000;
    if (!(m->type & DRM_MODE_TYPE_PREFERRED))
        r += 1;
    return r;
}

/* Set a want_w x want_h mode (refresh nearest want_hz) on the first connected connector (HDMI preferred)
 * with our first scanout buffer. If the display offers no mode of that size, its preferred mode is used
 * instead: read the real size back from tb_stats (entries 11, 12). 0 or a negative code; on failure the
 * display is left as it was. */
int tb_modeset(int fd, int want_w, int want_h, int want_hz, int nbuf) {
    if (T.ready || T.retired)
        return -EALREADY;
    if (fd < 0 || want_w < 64 || want_h < 64 || want_w > 4096 || want_h > 4096 || want_hz < 1 || want_hz > 240 ||
        nbuf < 2 || nbuf > TB_NBUF_MAX)
        return -EINVAL;
    uint64_t dumb = 0;
    if (get_cap(fd, DRM_CAP_DUMB_BUFFER, &dumb) || !dumb)
        return -ENOTSUP;
    tb_res r;
    int e = get_res(fd, &r);
    if (e)
        return e;
    struct drm_mode_modeinfo best_mode;
    memset(&best_mode, 0, sizeof best_mode);
    uint32_t best_conn = 0, best_crtc = 0, best_index = 0;
    int best_score = -1;                       /* HDMI 2, anything else 1; ties keep the first */
    for (uint32_t i = 0; i < lim(r.n_conns); i++) {
        struct drm_mode_get_connector c;
        struct drm_mode_modeinfo *modes = NULL;
        uint32_t encs[TB_MAXN];
        memset(encs, 0, sizeof encs);
        if (conn_modes(fd, r.conns[i], &c, &modes, encs, TB_MAXN))
            continue;
        int score = c.connector_type == DRM_MODE_CONNECTOR_HDMIA ? 2 : 1;
        if (c.connection != 1 || !c.count_modes || score <= best_score) {
            free(modes);
            continue;
        }
        int pick = -1, pick_rank = 0, pref = -1;
        for (uint32_t m = 0; m < c.count_modes; m++) {
            int rk = mode_rank(&modes[m], want_w, want_h, want_hz);
            if (rk >= 0 && (pick < 0 || rk < pick_rank)) {
                pick = (int)m;
                pick_rank = rk;
            }
            if (pref < 0 && (modes[m].type & DRM_MODE_TYPE_PREFERRED))
                pref = (int)m;
        }
        if (pick < 0)
            pick = pref >= 0 ? pref : 0;
        /* a CRTC this connector can drive: its current encoder's, else the first possible one */
        uint32_t crtc = 0, index = 0, nt = 0;
        uint32_t tryenc[TB_MAXN + 1];
        if (c.encoder_id)
            tryenc[nt++] = c.encoder_id;
        for (uint32_t k = 0; k < c.count_encoders && k < TB_MAXN; k++)
            tryenc[nt++] = encs[k];
        for (uint32_t k = 0; k < nt && !crtc; k++) {
            struct drm_mode_get_encoder en;
            if (get_enc(fd, tryenc[k], &en))
                continue;
            for (uint32_t j = 0; j < lim(r.n_crtcs) && en.crtc_id && !crtc; j++)
                if (r.crtcs[j] == en.crtc_id) {
                    crtc = en.crtc_id;
                    index = j;
                }
            for (uint32_t j = 0; j < lim(r.n_crtcs) && !crtc; j++)
                if (en.possible_crtcs & (1u << j)) {
                    crtc = r.crtcs[j];
                    index = j;
                }
        }
        if (crtc) {
            best_score = score;
            best_mode = modes[pick];
            best_conn = c.connector_id;
            best_crtc = crtc;
            best_index = index;
        }
        free(modes);
    }
    if (best_score < 0)
        return -ENOENT;
    uint32_t w = best_mode.hdisplay, h = best_mode.vdisplay;
    if (w < 64 || h < 64 || w > 4096 || h > 4096)
        return -EPROTO;
    for (int i = 0; i < nbuf; i++) {
        if ((e = make_buf(fd, w, h, &T.buf[i]))) {
            for (int j = 0; j < i; j++)
                free_buf(fd, &T.buf[j]);
            return e;
        }
    }
    struct drm_mode_crtc sc;
    memset(&sc, 0, sizeof sc);
    sc.set_connectors_ptr = (uint64_t)(uintptr_t)&best_conn;
    sc.count_connectors = 1;
    sc.crtc_id = best_crtc;
    sc.fb_id = T.buf[0].fb_id;
    sc.mode_valid = 1;
    sc.mode = best_mode;
    if ((e = xioctl(fd, DRM_IOCTL_MODE_SETCRTC, &sc))) {
        for (int i = 0; i < nbuf; i++)
            free_buf(fd, &T.buf[i]);
        return e;
    }
    T.fd = fd;
    T.nbuf = nbuf;
    T.crtc_id = best_crtc;
    T.crtc_index = best_index;
    T.conn_id = best_conn;
    T.orig_fb = 0;
    T.width = w;
    T.height = h;
    T.refresh = best_mode.vrefresh;
    T.front = 0;                               /* buffer 0 is on screen now */
    T.pending = -1;
    T.ready = 1;
    return 0;
}

/* ---- init: find SDL's CRTC (connected connector -> encoder -> CRTC showing a want_w x want_h mode) ---- */
int tb_init(int fd, int want_w, int want_h, int nbuf) {
    if (T.ready || T.retired)
        return -EALREADY;
    if (fd < 0 || want_w < 64 || want_h < 64 || want_w > 4096 || want_h > 4096 || nbuf < 2 || nbuf > TB_NBUF_MAX)
        return -EINVAL;
    int m = tb_is_master(fd);
    if (m < 0)
        return m;
    if (m == 0)
        return -EACCES;
    uint64_t dumb = 0;
    if (get_cap(fd, DRM_CAP_DUMB_BUFFER, &dumb) || !dumb)
        return -ENOTSUP;
    tb_res r;
    int e = get_res(fd, &r);
    if (e)
        return e;
    int best = -1;
    uint32_t best_crtc = 0, best_index = 0, best_fb = 0;
    for (uint32_t i = 0; i < lim(r.n_conns); i++) {
        struct drm_mode_get_connector c;
        struct drm_mode_modeinfo modes[TB_MAXMODES];
        if (get_conn(fd, r.conns[i], &c, modes) || c.connection != 1 || !c.encoder_id)
            continue;
        struct drm_mode_get_encoder en;
        if (get_enc(fd, c.encoder_id, &en) || !en.crtc_id)
            continue;
        struct drm_mode_crtc cr;
        if (get_crtc(fd, en.crtc_id, &cr) || !cr.mode_valid || !cr.fb_id)
            continue;
        if (cr.mode.hdisplay != (uint32_t)want_w || cr.mode.vdisplay != (uint32_t)want_h)
            continue;
        int score = (c.connector_type == DRM_MODE_CONNECTOR_HDMIA) ? 2 : 1;
        if (score <= best)
            continue;
        uint32_t idx = UINT32_MAX;
        for (uint32_t k = 0; k < lim(r.n_crtcs); k++)
            if (r.crtcs[k] == en.crtc_id)
                idx = k;
        if (idx == UINT32_MAX)
            continue;
        best = score;
        best_crtc = en.crtc_id;
        best_index = idx;
        best_fb = cr.fb_id;
        T.conn_id = c.connector_id;
    }
    if (best < 0)
        return -ENOENT;
    uint32_t fmt = 0, fw = 0, fh = 0;
    uint64_t mod = 0;
    if ((e = fb_format(fd, best_fb, &fmt, &mod, &fw, &fh)))
        return e;
    if (fmt != TB_XRGB8888)
        return -EPROTO;             /* a legacy flip may not change the format; ours would be XRGB8888 */
    for (int i = 0; i < nbuf; i++) {
        if ((e = make_buf(fd, (uint32_t)want_w, (uint32_t)want_h, &T.buf[i]))) {
            for (int j = 0; j < i; j++)
                free_buf(fd, &T.buf[j]);
            return e;
        }
    }
    T.fd = fd;
    T.nbuf = nbuf;
    T.crtc_id = best_crtc;
    T.crtc_index = best_index;
    T.orig_fb = best_fb;
    T.width = (uint32_t)want_w;
    T.height = (uint32_t)want_h;
    T.front = -1;
    T.pending = -1;
    T.ready = 1;
    return 0;
}

/* ---- vblank counters of our CRTC ------------------------------------------------------------------- */
static uint32_t vbl_bits(void) {
    if (T.crtc_index == 0)
        return 0;
    if (T.crtc_index == 1)
        return _DRM_VBLANK_SECONDARY;
    return (T.crtc_index << _DRM_VBLANK_HIGH_CRTC_SHIFT) & _DRM_VBLANK_HIGH_CRTC_MASK;
}

static int vbl_now(uint32_t *seq) {
    union drm_wait_vblank v;
    memset(&v, 0, sizeof v);
    v.request.type = (enum drm_vblank_seq_type)(_DRM_VBLANK_RELATIVE | vbl_bits());
    v.request.sequence = 0;
    int e = xioctl(T.fd, DRM_IOCTL_WAIT_VBLANK, &v);
    if (!e)
        *seq = T.last_seq = v.reply.sequence;
    return e;
}

static int vbl_wait_until(uint32_t target) {
    union drm_wait_vblank v;
    memset(&v, 0, sizeof v);
    v.request.type = (enum drm_vblank_seq_type)(_DRM_VBLANK_ABSOLUTE | vbl_bits());
    v.request.sequence = target;
    int e = xioctl(T.fd, DRM_IOCTL_WAIT_VBLANK, &v);
    if (!e) {
        T.vblank_waits++;
        T.last_seq = v.reply.sequence;
    }
    return e;
}

/* wait until the pending flip has (almost certainly) reached the screen; the flip ioctl re-checks */
static int settle_pending(void) {
    if (T.pending < 0)
        return 0;
    uint32_t now = 0;
    int e = vbl_now(&now);
    if (e)
        return e;
    if ((int32_t)(now - T.pending_seq) < 0 && (e = vbl_wait_until(T.pending_seq)))
        return e;
    T.front = T.pending;
    T.pending = -1;
    return 0;
}

static int flip_to(int b) {
    struct drm_mode_crtc_page_flip f;
    memset(&f, 0, sizeof f);
    f.crtc_id = T.crtc_id;
    f.fb_id = T.buf[b].fb_id;
    f.flags = 0;                    /* no DRM_MODE_PAGE_FLIP_EVENT: we never read events from SDL's fd */
    int e = 0;
    for (int tries = 0; tries < 40; tries++) {
        e = xioctl(T.fd, DRM_IOCTL_MODE_PAGE_FLIP, &f);
        if (e != -EBUSY)
            break;
        T.ebusy++;
        if (tries < 20) {
            sleep_us(250);          /* flip_done lands a few us after the vblank wake-up */
        } else {
            uint32_t now = 0;
            if (vbl_now(&now) || vbl_wait_until(now + 1))
                break;
        }
    }
    if (e)
        return e;
    uint32_t now = 0;
    if ((e = vbl_now(&now)))
        return e;
    T.pending = b;
    T.pending_seq = now + 1;
    T.flips++;
    return 0;
}

/* present one frame: copy src (XRGB8888, w x h, pitch bytes per row) into a free scanout buffer, wait for
 * the previous flip if it is still pending (this is what paces the engine to the display's refresh), flip */
int tb_present(const uint8_t *src, int pitch, int w, int h) {
    if (!T.ready)
        return -ENODEV;
    if (!src || ((uintptr_t)src & 3) || w != (int)T.width || h != (int)T.height || pitch < w * 4)
        return -EINVAL;
    double t0 = now_ms();
    int b = -1;
    for (int pass = 0; pass < 2 && b < 0; pass++) {
        for (int i = 0; i < T.nbuf; i++)
            if (i != T.front && i != T.pending) {
                b = i;
                break;
            }
        if (b < 0) {
            int e = settle_pending();
            if (e) {
                T.errors++;
                return T.last_err = e;
            }
        }
    }
    if (b < 0) {
        T.errors++;
        return T.last_err = -EDEADLK;
    }
    tb_copy_rows(T.buf[b].map, (int)T.buf[b].pitch, src, pitch, w * 4, h);
    double t1 = now_ms();
    int e = settle_pending();
    double t2 = now_ms();
    if (!e)
        e = flip_to(b);
    double t3 = now_ms();
    if (e) {
        T.errors++;
        return T.last_err = e;
    }
    T.frames++;
    T.copy_ms += t1 - t0;
    T.wait_ms += t2 - t1;
    T.flip_ms += t3 - t2;
    return 0;
}

/* hand presenting back to SDL: make sure our last flip is on screen so SDL's next flip is not refused.
 * Our buffers stay allocated (one may still be on screen until SDL's flip lands); process exit frees them. */
int tb_handback(void) {
    if (!T.ready)
        return -ENODEV;
    int e = settle_pending();
    T.ready = 0;
    T.retired = 1;
    return e;
}

/* frames, flips, ebusy, vblank_waits, errors, copy_ms, wait_ms, flip_ms, last_err, crtc_id, crtc_index,
 * width, height, pitch, last vblank seq, refresh Hz (v3) */
int tb_stats(double *out, int n) {
    double v[] = { (double)T.frames, (double)T.flips, (double)T.ebusy, (double)T.vblank_waits, (double)T.errors,
                   T.copy_ms, T.wait_ms, T.flip_ms, (double)T.last_err, (double)T.crtc_id, (double)T.crtc_index,
                   (double)T.width, (double)T.height, (double)(T.nbuf ? T.buf[0].pitch : 0), (double)T.last_seq, (double)T.refresh };
    int k = (int)(sizeof v / sizeof v[0]);
    if (!out || n <= 0)
        return k;
    for (int i = 0; i < n && i < k; i++)
        out[i] = v[i];
    return k;
}

/* ---- hardware render scale: an overlay plane the HVS scales (v2) ----------------------------------- */
static struct {
    int ready;
    uint32_t plane_id, sw, sh, dw, dh;
    int32_t dx, dy;
    tb_buf buf[3];
    int on_screen, committing, queued;     /* buffer indices, -1 = none */
    int stop, started;
    pthread_t th;
    pthread_mutex_t mu;
    pthread_cond_t cv;
    uint64_t presents, commits, errors;
    double copy_ms, wait_ms, commit_ms;
    int last_err;
} P = { .on_screen = -1, .committing = -1, .queued = -1, .mu = PTHREAD_MUTEX_INITIALIZER, .cv = PTHREAD_COND_INITIALIZER };

static int set_plane(int b) {
#ifdef TB_TEST_FAKE
    for (int i = 0; i < 8; i++) {           /* ~2 ms "until the next vblank", watching for writers */
        int c = __atomic_load_n(&fake_copying, __ATOMIC_SEQ_CST);
        if (c >= 0 && (c == b || c == __atomic_load_n(&P.on_screen, __ATOMIC_SEQ_CST)))
            __atomic_add_fetch(&fake_violations, 1, __ATOMIC_RELAXED);
        sleep_us(250);
    }
    return 0;
#endif
    struct drm_mode_set_plane s;
    memset(&s, 0, sizeof s);
    s.plane_id = P.plane_id;
    s.crtc_id = T.crtc_id;
    s.fb_id = b >= 0 ? P.buf[b].fb_id : 0;  /* fb 0 = switch the plane off */
    s.crtc_x = P.dx;
    s.crtc_y = P.dy;
    s.crtc_w = P.dw;
    s.crtc_h = P.dh;
    s.src_x = 0;
    s.src_y = 0;
    s.src_w = P.sw << 16;                   /* 16.16 fixed point */
    s.src_h = P.sh << 16;
    return xioctl(T.fd, DRM_IOCTL_MODE_SETPLANE, &s);
}

static void *plane_worker(void *arg) {
    (void)arg;
    pthread_mutex_lock(&P.mu);
    for (;;) {
        while (!P.stop && P.queued < 0)
            pthread_cond_wait(&P.cv, &P.mu);
        if (P.stop)
            break;
        int b = P.queued;
        P.queued = -1;
        P.committing = b;
        pthread_cond_broadcast(&P.cv);      /* a presenter waiting on the queue slot can go on */
        pthread_mutex_unlock(&P.mu);
        double t0 = now_ms();
        int e = set_plane(b);               /* blocks until the new buffer is on screen */
        double t1 = now_ms();
        pthread_mutex_lock(&P.mu);
        P.committing = -1;
        if (e) {
            P.errors++;
            P.last_err = e;
        } else {
            __atomic_store_n(&P.on_screen, b, __ATOMIC_SEQ_CST);
            P.commits++;
            P.commit_ms += t1 - t0;
        }
        pthread_cond_broadcast(&P.cv);
    }
    pthread_mutex_unlock(&P.mu);
    return NULL;
}

/* Put a sw x sh frame on a free overlay plane scaled to (dx, dy, dw, dh) on TURBO's CRTC. Needs tb_init.
 * Returns the plane id (> 0) or a negative code. The plane shows black until the first tb_plane_present. */
int tb_plane_init(int sw, int sh, int dx, int dy, int dw, int dh) {
#ifdef TB_TEST_FAKE
    T.ready = 1;
#endif
    if (!T.ready)
        return -ENODEV;
    if (P.ready)
        return -EALREADY;
    if (sw < 16 || sh < 16 || sw > 2048 || sh > 2048 || dw < 16 || dh < 16 || dw > 4096 || dh > 4096 ||
        dx < -4096 || dy < -4096 || dx > 4096 || dy > 4096)
        return -EINVAL;
    uint32_t planes[TB_MAXN];
    struct drm_mode_get_plane_res pr;
    memset(&pr, 0, sizeof pr);
    pr.plane_id_ptr = (uint64_t)(uintptr_t)planes;
    pr.count_planes = TB_MAXN;
    int e = xioctl(T.fd, DRM_IOCTL_MODE_GETPLANERESOURCES, &pr);
    uint32_t pick = 0;
#ifdef TB_TEST_FAKE
    e = 0;
    pr.count_planes = 0;
    pick = 99;
#endif
    if (e)
        return e;
    for (uint32_t i = 0; i < lim(pr.count_planes) && !pick; i++) {
        uint32_t fmts[64];
        struct drm_mode_get_plane p;
        memset(&p, 0, sizeof p);
        p.plane_id = planes[i];
        p.format_type_ptr = (uint64_t)(uintptr_t)fmts;
        p.count_format_types = 64;
        if (xioctl(T.fd, DRM_IOCTL_MODE_GETPLANE, &p) || p.crtc_id || p.fb_id || !(p.possible_crtcs & (1u << T.crtc_index)))
            continue;
        uint32_t nf = p.count_format_types < 64 ? p.count_format_types : 64;
        for (uint32_t f = 0; f < nf; f++)
            if (fmts[f] == TB_XRGB8888)
                pick = p.plane_id;
    }
    if (!pick)
        return -ENOENT;
    for (int i = 0; i < 3; i++) {
        if ((e = make_buf(T.fd, (uint32_t)sw, (uint32_t)sh, &P.buf[i]))) {
            for (int j = 0; j < i; j++)
                free_buf(T.fd, &P.buf[j]);
            return e;
        }
    }
    P.plane_id = pick;
    P.sw = (uint32_t)sw;
    P.sh = (uint32_t)sh;
    P.dx = dx;
    P.dy = dy;
    P.dw = (uint32_t)dw;
    P.dh = (uint32_t)dh;
    P.on_screen = P.committing = P.queued = -1;
    P.stop = 0;
    P.presents = P.commits = P.errors = 0;
    P.copy_ms = P.wait_ms = P.commit_ms = 0;
    P.last_err = 0;
    if ((e = set_plane(0))) {             /* first commit synchronously: proves the plane + scaling work */
        for (int i = 0; i < 3; i++)
            free_buf(T.fd, &P.buf[i]);
        return e;
    }
    P.on_screen = 0;
    if (pthread_create(&P.th, NULL, plane_worker, NULL)) {
        set_plane(-1);
        for (int i = 0; i < 3; i++)
            free_buf(T.fd, &P.buf[i]);
        return -EAGAIN;
    }
    P.started = 1;
    P.ready = 1;
    return (int)pick;
}

/* copy a sw x sh XRGB8888 frame into a free plane buffer and queue it for the worker */
int tb_plane_present(const uint8_t *src, int pitch, int w, int h) {
    if (!P.ready)
        return -ENODEV;
    if (!src || ((uintptr_t)src & 3) || w != (int)P.sw || h != (int)P.sh || pitch < w * 4)
        return -EINVAL;
    double t0 = now_ms();
    pthread_mutex_lock(&P.mu);
    while (P.queued >= 0 && !P.stop)
        pthread_cond_wait(&P.cv, &P.mu);
    int err = P.last_err;
    int b = -1;
    for (int i = 0; i < 3; i++)
        if (i != P.on_screen && i != P.committing) {
            b = i;
            break;
        }
    pthread_mutex_unlock(&P.mu);
    if (err)
        return err;                       /* a commit failed: let the caller switch the plane off */
    if (b < 0)
        return -EDEADLK;
    double t1 = now_ms();
#ifdef TB_TEST_FAKE
    __atomic_store_n(&fake_copying, b, __ATOMIC_SEQ_CST);
#endif
    tb_copy_rows(P.buf[b].map, (int)P.buf[b].pitch, src, pitch, w * 4, h);
#ifdef TB_TEST_FAKE
    __atomic_store_n(&fake_copying, -1, __ATOMIC_SEQ_CST);
#endif
    double t2 = now_ms();
    pthread_mutex_lock(&P.mu);
    P.queued = b;
    P.presents++;
    P.wait_ms += t1 - t0;
    P.copy_ms += t2 - t1;
    pthread_cond_broadcast(&P.cv);
    pthread_mutex_unlock(&P.mu);
    return 0;
}

/* stop the worker, switch the plane off (blocking: afterwards none of its buffers is scanned out), free */
int tb_plane_off(void) {
    if (!P.ready)
        return -ENODEV;
    pthread_mutex_lock(&P.mu);
    P.stop = 1;
    pthread_cond_broadcast(&P.cv);
    pthread_mutex_unlock(&P.mu);
    if (P.started)
        pthread_join(P.th, NULL);
    P.started = 0;
    int e = set_plane(-1);
    if (!e)
        for (int i = 0; i < 3; i++)
            free_buf(T.fd, &P.buf[i]);    /* if switching off failed, keep them: one may still be on screen */
    P.ready = 0;
    return e;
}

/* presents, commits, errors, copy_ms, wait_ms, commit_ms, last_err, plane_id, sw, sh */
int tb_plane_stats(double *out, int n) {
    pthread_mutex_lock(&P.mu);
    double v[] = { (double)P.presents, (double)P.commits, (double)P.errors, P.copy_ms, P.wait_ms, P.commit_ms,
                   (double)P.last_err, (double)P.plane_id, (double)P.sw, (double)P.sh };
    pthread_mutex_unlock(&P.mu);
    int k = (int)(sizeof v / sizeof v[0]);
    if (!out || n <= 0)
        return k;
    for (int i = 0; i < n && i < k; i++)
        out[i] = v[i];
    return k;
}

/* ---- preview (v5): a small JPEG of the latest frame, for the browser stream ---------------------------
 * When someone watched within the last 2 s (tb_preview_touch) and the preview interval has passed, the
 * compositor worker box-averages the frame it just copied (before clearing it) into a small XRGB buffer.
 * An encoder thread turns that into a JPEG with libjpeg-turbo (fed BGRX directly: no conversion) on its
 * own core, and tb_preview_jpeg() hands the newest one to whoever asks. Nobody watching = no cost.
 * Built without libjpeg (-DTB_NO_JPEG) the preview functions return -ENOTSUP. */
#ifndef TB_NO_JPEG
#include <setjmp.h>
#include <jpeglib.h>
#endif

static struct {
    int ready, stop, started;
    int pw, ph, quality, every_ms;
    uint8_t *small[2];               /* downscaled frames, XRGB, pw*ph*4 */
    int pending, encoding;           /* index waiting for the encoder / being encoded, -1 none */
    uint8_t *jpeg;                   /* the newest JPEG */
    size_t jpeg_len, jpeg_cap;
    uint64_t seq;                    /* bumps with every new JPEG */
    double last_touch, last_grab;
    uint64_t grabs, encodes, enc_errors;
    double down_ms, enc_ms;
    pthread_t th;
    pthread_mutex_t mu;
    pthread_cond_t cv;
} V = { .pending = -1, .encoding = -1, .mu = PTHREAD_MUTEX_INITIALIZER, .cv = PTHREAD_COND_INITIALIZER };

/* area average of src (w x h, XRGB) into dst (pw x ph, XRGB): each output pixel is the mean (rounded down)
 * of the source block it covers, so thin lines fade instead of vanishing. No division per pixel: block
 * bounds are tabled once per size and the mean uses an exact reciprocal (on the EYESY's 32-bit build a
 * division per pixel was a library call; 1280x720 -> 640x360 took ~75 ms). Exact halving and 1:1 are
 * special-cased. */
static struct {
    int w, h, pw, ph;
    int *x0, *x1, *y0, *y1;
} DS = { -1, -1, -1, -1, NULL, NULL, NULL, NULL };
static uint32_t RECIP[256];                 /* ceil(2^24 / n): floor(s / n) = (s * RECIP[n]) >> 24 */

static int ds_prepare(int w, int h, int pw, int ph) {
    if (!RECIP[1])
        for (uint32_t n = 1; n < 256; n++)
            RECIP[n] = ((1u << 24) + n - 1) / n;
    if (DS.w == w && DS.h == h && DS.pw == pw && DS.ph == ph)
        return 0;
    free(DS.x0);
    free(DS.y0);
    DS.x0 = malloc(sizeof(int) * 2 * (size_t)pw);
    DS.y0 = malloc(sizeof(int) * 2 * (size_t)ph);
    if (!DS.x0 || !DS.y0) {
        free(DS.x0);
        free(DS.y0);
        DS.x0 = DS.y0 = NULL;
        DS.w = -1;
        return -ENOMEM;
    }
    DS.x1 = DS.x0 + pw;
    DS.y1 = DS.y0 + ph;
    for (int x = 0; x < pw; x++) {
        int a = (int)((int64_t)x * w / pw), b = (int)((int64_t)(x + 1) * w / pw);
        DS.x0[x] = a;
        DS.x1[x] = b > a ? b : a + 1;
    }
    for (int y = 0; y < ph; y++) {
        int a = (int)((int64_t)y * h / ph), b = (int)((int64_t)(y + 1) * h / ph);
        DS.y0[y] = a;
        DS.y1[y] = b > a ? b : a + 1;
    }
    DS.w = w;
    DS.h = h;
    DS.pw = pw;
    DS.ph = ph;
    return 0;
}

static void downscale(uint8_t *dst, int pw, int ph, const uint8_t *src, int pitch, int w, int h) {
    if (w == pw && h == ph) {
        for (int y = 0; y < ph; y++)
            memcpy(dst + (size_t)y * pw * 4, src + (size_t)y * pitch, (size_t)pw * 4);
        return;
    }
    if (w == 2 * pw && h == 2 * ph) {                     /* 1280x720 -> 640x360: packed 2x2 means */
        for (int y = 0; y < ph; y++) {
            const uint32_t *r0 = (const uint32_t *)(src + (size_t)(2 * y) * pitch);
            const uint32_t *r1 = (const uint32_t *)(src + (size_t)(2 * y + 1) * pitch);
            uint32_t *out = (uint32_t *)(dst + (size_t)y * pw * 4);
            for (int x = 0; x < pw; x++) {
                uint32_t a = r0[2 * x], b = r0[2 * x + 1], c = r1[2 * x], d = r1[2 * x + 1];
                uint32_t rb = (a & 0xFF00FF) + (b & 0xFF00FF) + (c & 0xFF00FF) + (d & 0xFF00FF);
                uint32_t g = (a & 0xFF00) + (b & 0xFF00) + (c & 0xFF00) + (d & 0xFF00);
                out[x] = ((rb >> 2) & 0xFF00FF) | ((g >> 2) & 0xFF00);
            }
        }
        return;
    }
    if (ds_prepare(w, h, pw, ph))
        return;
    for (int y = 0; y < ph; y++) {
        int y0 = DS.y0[y], y1 = DS.y1[y];
        uint32_t *out = (uint32_t *)(dst + (size_t)y * pw * 4);
        for (int x = 0; x < pw; x++) {
            int x0 = DS.x0[x], x1 = DS.x1[x];
            uint32_t r = 0, g = 0, b = 0;
            for (int yy = y0; yy < y1; yy++) {
                const uint32_t *row = (const uint32_t *)(src + (size_t)yy * pitch);
                for (int xx = x0; xx < x1; xx++) {
                    uint32_t p = row[xx];
                    r += (p >> 16) & 0xff;
                    g += (p >> 8) & 0xff;
                    b += p & 0xff;
                }
            }
            uint32_t n = (uint32_t)((x1 - x0) * (y1 - y0));
            if (n <= 255) {
                uint64_t m = RECIP[n];                    /* exact for s <= 255 * 255 */
                out[x] = (uint32_t)(((r * m) >> 24) << 16 | ((g * m) >> 24) << 8 | ((b * m) >> 24));
            } else {
                out[x] = ((r / n) << 16) | ((g / n) << 8) | (b / n);
            }
        }
    }
}

/* the OSD layer (ARGB, straight alpha, on screen unscaled at the top left of an sw x sh screen) over a
 * pw x ph preview of that whole screen, so the preview shows what HDMI shows. Each preview pixel takes the
 * alpha-weighted mean of the OSD pixels in its box (the parts of the box outside the OSD count as
 * transparent), blended over the frame's pixel; boxes that are all transparent are left alone. */
static int OB_pw = -1, OB_sw = -1, OB_x0[4097], OB_x1[4097];   /* osd_blend's column boxes (worker only) */

static inline uint32_t div_box(uint32_t v, uint32_t d) {
    return d == 1020 ? v / 1020 : v / d;                  /* 2x2 boxes (the usual 1280x720 -> 640x360): a
                                                             constant, so a multiply instead of a divide */
}

static void osd_blend(uint8_t *dst, int pw, int ph, const uint8_t *osd, int opitch, int ow, int oh, int sw, int sh) {
    if (!osd || ow < 1 || oh < 1 || sw < 1 || sh < 1 || ow > sw || oh > sh)
        return;
    if (pw < 1 || ph < 1 || pw > 4096 || ph > 4096 || sw > 16384 || sh > 16384)
        return;                                          /* keeps the bounds maths below in 32 bits: no
                                                            library calls for 64-bit division on the EYESY */
    int xe = (ow * pw + sw - 1) / sw;                    /* the preview columns and rows the OSD reaches */
    int ye = (oh * ph + sh - 1) / sh;
    if (xe > pw)
        xe = pw;
    if (ye > ph)
        ye = ph;
    if (OB_pw != pw || OB_sw != sw) {
        for (int x = 0; x < pw; x++) {
            int a = x * sw / pw, b = (x + 1) * sw / pw;
            OB_x0[x] = a;
            OB_x1[x] = b > a ? b : a + 1;
        }
        OB_pw = pw;
        OB_sw = sw;
    }
    for (int y = 0; y < ye; y++) {
        int y0 = y * sh / ph, y1 = (y + 1) * sh / ph;
        if (y1 <= y0)
            y1 = y0 + 1;
        int oy1 = y1 < oh ? y1 : oh;
        /* most of the OSD is transparent: a preview row whose OSD rows have no alpha anywhere costs one
         * load + OR per OSD pixel and is left alone (a transparent OSD leaves the frame as it was) */
        uint32_t any = 0;
        for (int yy = y0; yy < oy1 && !(any & 0xFF000000u); yy++) {
            const uint32_t *row = (const uint32_t *)(osd + (size_t)yy * opitch);
            for (int xx = 0; xx < ow && !(any & 0xFF000000u); xx += 16) {
                int e = xx + 16 < ow ? xx + 16 : ow;
                for (int i = xx; i < e; i++)
                    any |= row[i];
            }
        }
        if (!(any & 0xFF000000u))
            continue;
        uint32_t *out = (uint32_t *)(dst + (size_t)y * pw * 4);
        for (int x = 0; x < xe; x++) {
            int x0 = OB_x0[x], x1 = OB_x1[x];
            int ox1 = x1 < ow ? x1 : ow;
            uint32_t n = (uint32_t)((x1 - x0) * (y1 - y0));
            if (n > 32768)                                  /* 32-bit sums below; no real screen gets here */
                return;
            uint32_t box = 0;                               /* the same shortcut per box */
            for (int yy = y0; yy < oy1; yy++) {
                const uint32_t *row = (const uint32_t *)(osd + (size_t)yy * opitch);
                for (int xx = x0; xx < ox1; xx++)
                    box |= row[xx];
            }
            if (!(box & 0xFF000000u))
                continue;
            uint32_t sa = 0, sr = 0, sg = 0, sb = 0;
            for (int yy = y0; yy < oy1; yy++) {
                const uint32_t *row = (const uint32_t *)(osd + (size_t)yy * opitch);
                for (int xx = x0; xx < ox1; xx++) {
                    uint32_t p = row[xx], a = p >> 24;
                    sa += a;
                    sr += ((p >> 16) & 0xff) * a;
                    sg += ((p >> 8) & 0xff) * a;
                    sb += (p & 0xff) * a;
                }
            }
            /* out = mean premultiplied colour + frame * (1 - mean alpha), in 1/(255 n) units, floored */
            uint32_t d = 255 * n, k = d - sa, f = out[x];
            uint32_t r = div_box(sr + ((f >> 16) & 0xff) * k, d);
            uint32_t g = div_box(sg + ((f >> 8) & 0xff) * k, d);
            uint32_t b = div_box(sb + (f & 0xff) * k, d);
            out[x] = (r << 16) | (g << 8) | b;
        }
    }
}

/* called by the compositor worker with the frame it just copied (not yet cleared) and the OSD layer that goes
 * on screen with it (NULL = none) */
static void preview_grab(const uint8_t *src, int pitch, int w, int h,
                         const uint8_t *osd, int opitch, int ow, int oh) {
    if (!V.ready)
        return;
    double now = now_ms();
    pthread_mutex_lock(&V.mu);
    int want = (now - V.last_touch) < 2000.0 && (now - V.last_grab) >= V.every_ms;
    int target = V.encoding == 0 ? 1 : 0;           /* never the buffer being encoded; may replace `pending` */
    if (want) {
        V.last_grab = now;
        if (V.pending == target)
            V.pending = -1;                            /* about to be overwritten with a newer frame */
    }
    pthread_mutex_unlock(&V.mu);
    if (!want)
        return;
    double t0 = now_ms();
    downscale(V.small[target], V.pw, V.ph, src, pitch, w, h);
    if (osd)
        osd_blend(V.small[target], V.pw, V.ph, osd, opitch, ow, oh, (int)T.width, (int)T.height);
    double t1 = now_ms();
    pthread_mutex_lock(&V.mu);
    V.pending = target;
    V.grabs++;
    V.down_ms += t1 - t0;
    pthread_cond_broadcast(&V.cv);
    pthread_mutex_unlock(&V.mu);
}

#ifndef TB_NO_JPEG
struct jerr {
    struct jpeg_error_mgr pub;
    jmp_buf jb;
};

static void jerr_exit(j_common_ptr c) {
    longjmp(((struct jerr *)c->err)->jb, 1);       /* libjpeg's default would exit() the whole engine */
}

static void jerr_quiet(j_common_ptr c, int level) {
    (void)c;
    (void)level;
}

/* XRGB little-endian = B G R X in memory = libjpeg-turbo's JCS_EXT_BGRX */
static int encode(const uint8_t *px, int w, int h, int quality, unsigned char **out, unsigned long *len) {
    struct jpeg_compress_struct c;
    struct jerr je;
    *out = NULL;
    *len = 0;
    c.err = jpeg_std_error(&je.pub);
    je.pub.error_exit = jerr_exit;
    je.pub.emit_message = jerr_quiet;
    if (setjmp(je.jb)) {
        jpeg_destroy_compress(&c);
        free(*out);
        *out = NULL;
        return -EIO;
    }
    jpeg_create_compress(&c);
    jpeg_mem_dest(&c, out, len);
    c.image_width = (JDIMENSION)w;
    c.image_height = (JDIMENSION)h;
    c.input_components = 4;
    c.in_color_space = JCS_EXT_BGRX;
    jpeg_set_defaults(&c);
    jpeg_set_quality(&c, quality, TRUE);
    c.dct_method = JDCT_IFAST;
    jpeg_start_compress(&c, TRUE);
    while (c.next_scanline < c.image_height) {
        JSAMPROW row = (JSAMPROW)(px + (size_t)c.next_scanline * w * 4);
        jpeg_write_scanlines(&c, &row, 1);
    }
    jpeg_finish_compress(&c);
    jpeg_destroy_compress(&c);
    return 0;
}
#endif

static void *preview_worker(void *arg) {
    (void)arg;
    pthread_mutex_lock(&V.mu);
    for (;;) {
        while (!V.stop && V.pending < 0)
            pthread_cond_wait(&V.cv, &V.mu);
        if (V.stop)
            break;
        int b = V.pending;
        V.pending = -1;
        V.encoding = b;
        pthread_mutex_unlock(&V.mu);
        double t0 = now_ms();
        unsigned char *jp = NULL;
        unsigned long jl = 0;
        int e = -ENOTSUP;
#ifndef TB_NO_JPEG
        e = encode(V.small[b], V.pw, V.ph, V.quality, &jp, &jl);
#endif
        double t1 = now_ms();
        pthread_mutex_lock(&V.mu);
        V.encoding = -1;
        if (!e && jp && jl) {
            if (jl > V.jpeg_cap) {
                uint8_t *nb = realloc(V.jpeg, jl);
                if (nb) {
                    V.jpeg = nb;
                    V.jpeg_cap = jl;
                }
            }
            if (jl <= V.jpeg_cap) {
                memcpy(V.jpeg, jp, jl);
                V.jpeg_len = jl;
                V.seq++;
                V.encodes++;
                V.enc_ms += t1 - t0;
            }
        } else {
            V.enc_errors++;
        }
        free(jp);
        pthread_cond_broadcast(&V.cv);
    }
    pthread_mutex_unlock(&V.mu);
    return NULL;
}

/* pw x ph preview at `quality`, at most one every `every_ms` while someone watches */
int tb_preview_init(int pw, int ph, int quality, int every_ms) {
#ifdef TB_NO_JPEG
    (void)pw; (void)ph; (void)quality; (void)every_ms;
    return -ENOTSUP;
#else
    if (V.ready)
        return -EALREADY;
    if (pw < 16 || ph < 16 || pw > 1920 || ph > 1080 || quality < 10 || quality > 95 || every_ms < 10 || every_ms > 5000)
        return -EINVAL;
    for (int i = 0; i < 2; i++) {
        V.small[i] = calloc((size_t)pw * ph, 4);
        if (!V.small[i]) {
            free(V.small[0]);
            V.small[0] = NULL;
            return -ENOMEM;
        }
    }
    V.pw = pw;
    V.ph = ph;
    V.quality = quality;
    V.every_ms = every_ms;
    V.stop = 0;
    V.last_touch = -1e9;
    if (pthread_create(&V.th, NULL, preview_worker, NULL))
        return -EAGAIN;
    V.started = 1;
    V.ready = 1;
    return 0;
#endif
}

/* someone is watching: keep previews coming for the next 2 s */
void tb_preview_touch(void) {
    pthread_mutex_lock(&V.mu);
    V.last_touch = now_ms();
    pthread_mutex_unlock(&V.mu);
}

/* the newest JPEG if it is newer than *seq: copied into out (cap bytes), *seq updated, its length returned.
 * 0 = nothing newer yet; -ENOSPC = out too small (the needed size is in *need); -ENOTSUP = no preview */
int tb_preview_jpeg(uint8_t *out, int cap, uint64_t *seq, int *need) {
    if (!V.ready)
        return -ENOTSUP;
    if (!out || !seq || cap <= 0)
        return -EINVAL;
    pthread_mutex_lock(&V.mu);
    V.last_touch = now_ms();
    int r = 0;
    if (V.seq != *seq && V.jpeg_len) {
        if (V.jpeg_len > (size_t)cap) {
            if (need)
                *need = (int)V.jpeg_len;
            r = -ENOSPC;
        } else {
            memcpy(out, V.jpeg, V.jpeg_len);
            *seq = V.seq;
            r = (int)V.jpeg_len;
        }
    }
    pthread_mutex_unlock(&V.mu);
    return r;
}

/* grabs, encodes, enc_errors, downscale ms total, encode ms total, last jpeg bytes, seq, pw, ph */
int tb_preview_stats(double *out, int n) {
    pthread_mutex_lock(&V.mu);
    double v[] = { (double)V.grabs, (double)V.encodes, (double)V.enc_errors, V.down_ms, V.enc_ms,
                   (double)V.jpeg_len, (double)V.seq, (double)V.pw, (double)V.ph };
    pthread_mutex_unlock(&V.mu);
    int k = (int)(sizeof v / sizeof v[0]);
    if (!out || n <= 0)
        return k;
    for (int i = 0; i < n && i < k; i++)
        out[i] = v[i];
    return k;
}

int tb_preview_stop(void) {
    if (!V.ready)
        return -ENODEV;
    pthread_mutex_lock(&V.mu);
    V.stop = 1;
    pthread_cond_broadcast(&V.cv);
    pthread_mutex_unlock(&V.mu);
    if (V.started)
        pthread_join(V.th, NULL);
    V.started = 0;
    V.ready = 0;
    return 0;
}

/* ---- compositor (v4): one worker thread owns the screen through atomic commits -----------------------
 * The engine hands over a finished frame of any size up to the display's (the display hardware scales it
 * to full screen on the primary plane), asks for that surface to be cleared once copied, and optionally
 * passes an ARGB OSD surface for an overlay plane on top. The worker copies both into free scanout
 * buffers, clears the frame surface, waits for the previous flip if it is still in flight, then makes ONE
 * nonblocking atomic commit (mode layer + OSD layer together) and takes the next job. The engine draws
 * its next frame into its other surface meanwhile.
 *   tb_comp_present() waits until the worker has handed the previous job to the display (so one frame is
 *     in the worker and at most one on its way to the screen), queues this one and returns. Its interval is
 *     the frame's swap interval: it lands that many vblanks after the previous flip (1 = the next one), so
 *     a mode at 30 fps on a 60 Hz display shows every frame for exactly two refreshes.
 *   tb_comp_sync() waits until the worker no longer touches the last frame surface (persist mode draws
 *     into the same surface again).
 * Needs tb_modeset first (the mode is set the legacy way; then this switches the fd to atomic). */

#define CP_NBUF 3
#define TB_ARGB8888 0x34325241u    /* fourcc 'AR24' */

typedef struct {
    uint32_t fb_id, crtc_id, src_x, src_y, src_w, src_h, crtc_x, crtc_y, crtc_w, crtc_h, zpos;
    uint64_t zpos_max;
} plane_props;

typedef struct {
    tb_buf buf[CP_NBUF];
    uint32_t w, h;
    int n;
} layer;

static struct {
    int ready, stop, started;
    uint32_t primary, osd;       /* plane ids (osd 0 = no OSD plane) */
    plane_props pp[2];
    layer lay[2];                /* 0 = mode layer (XRGB, render size), 1 = OSD (ARGB) */
    layer retired;               /* the previous mode-layer set after a size change */
    uint64_t retire_after;       /* free `retired` once this many flips completed */
    int shown[2], flying[2];     /* buffer index on screen / in the commit in flight, per layer; -1 none */
    int osd_enabled;             /* the OSD plane was enabled by the last commit */
    int flip_pending;
    uint64_t issued, completed;  /* commits issued / flips completed */
    pthread_t th;
    pthread_mutex_t mu;
    pthread_cond_t cv;
    int has_job, src_busy, job_busy;
    uint8_t *src;
    int pitch, w, h, clear;
    uint32_t clear_xrgb;
    const uint8_t *osd_src;
    int osd_pitch, osd_w, osd_h, osd_show;
    int interval;                /* the queued job's swap interval: vblanks from the previous flip to its own */
    uint64_t frames, errors;
    double copy_ms, clear_ms, osd_ms, flipwait_ms, commit_ms, submit_wait_ms, last_flip_ms;
    double last_commit_ms;       /* when the last commit was asked for (the engine's frame scheduling measures how
                                    long before its vblank a commit must come) */
    int last_err;
    uint32_t last_seq;
} C = { .shown = { -1, -1 }, .flying = { -1, -1 }, .mu = PTHREAD_MUTEX_INITIALIZER, .cv = PTHREAD_COND_INITIALIZER };

#ifdef TB_TEST_FAKE
/* test build: commits "reach the screen" ~2 ms later; the worker checks nobody writes a buffer that is on
 * screen or in flight, and that the engine never writes the surface the worker is reading */
static double fake_flip_at;
static volatile long fake_comp_violations;
static int fake_copying_layer0 = -1;
#define FAKE_VBL_MS 2.0                  /* the fake display's refresh: a commit lands ~one of them later */
static long fake_commits_at[5];          /* commits per swap interval 1..4 */
static double fake_min_gap_ms[5] = { 1e9, 1e9, 1e9, 1e9, 1e9 };   /* shortest previous-flip -> commit gap */
#endif

static void layer_free(layer *l) {
    for (int i = 0; i < l->n; i++)
        free_buf(T.fd, &l->buf[i]);
    memset(l, 0, sizeof *l);
}

static int layer_alloc(layer *l, uint32_t w, uint32_t h, uint32_t depth) {
    memset(l, 0, sizeof *l);
    for (int i = 0; i < CP_NBUF; i++) {
        int e = make_buf_d(T.fd, w, h, depth, &l->buf[i]);
        if (e) {
            l->n = i;
            layer_free(l);
            return e;
        }
    }
    l->n = CP_NBUF;
    l->w = w;
    l->h = h;
    return 0;
}

static int free_index(int li) {
    for (int i = 0; i < C.lay[li].n; i++)
        if (i != C.shown[li] && i != C.flying[li])
            return i;
    return -1;
}

static void fill32(uint8_t *dst, int pitch, int w, int h, uint32_t v) {
    uint32_t *row0 = (uint32_t *)dst;
    for (int x = 0; x < w; x++)
        row0[x] = v;
    for (int y = 1; y < h; y++)
        memcpy(dst + (size_t)y * pitch, dst, (size_t)w * 4);
}

/* ---- property ids, plane lookup, atomic requests ---- */
static int plane_props_get(uint32_t plane, plane_props *pp, int *type) {
    uint32_t ids[64];
    uint64_t vals[64];
    struct drm_mode_obj_get_properties g;
    memset(&g, 0, sizeof g);
    g.props_ptr = (uint64_t)(uintptr_t)ids;
    g.prop_values_ptr = (uint64_t)(uintptr_t)vals;
    g.count_props = 64;
    g.obj_id = plane;
    g.obj_type = DRM_MODE_OBJECT_PLANE;
    int e = xioctl(T.fd, DRM_IOCTL_MODE_OBJ_GETPROPERTIES, &g);
    if (e)
        return e;
    memset(pp, 0, sizeof *pp);
    if (type)
        *type = -1;
    uint32_t n = g.count_props < 64 ? g.count_props : 64;
    for (uint32_t i = 0; i < n; i++) {
        uint64_t range[2] = { 0, 0 };
        struct drm_mode_get_property p;
        memset(&p, 0, sizeof p);
        p.prop_id = ids[i];
        p.values_ptr = (uint64_t)(uintptr_t)range;
        p.count_values = 2;
        if (xioctl(T.fd, DRM_IOCTL_MODE_GETPROPERTY, &p))
            continue;
        p.name[DRM_PROP_NAME_LEN - 1] = 0;
        const char *nm = p.name;
        if (!strcmp(nm, "FB_ID")) pp->fb_id = ids[i];
        else if (!strcmp(nm, "CRTC_ID")) pp->crtc_id = ids[i];
        else if (!strcmp(nm, "SRC_X")) pp->src_x = ids[i];
        else if (!strcmp(nm, "SRC_Y")) pp->src_y = ids[i];
        else if (!strcmp(nm, "SRC_W")) pp->src_w = ids[i];
        else if (!strcmp(nm, "SRC_H")) pp->src_h = ids[i];
        else if (!strcmp(nm, "CRTC_X")) pp->crtc_x = ids[i];
        else if (!strcmp(nm, "CRTC_Y")) pp->crtc_y = ids[i];
        else if (!strcmp(nm, "CRTC_W")) pp->crtc_w = ids[i];
        else if (!strcmp(nm, "CRTC_H")) pp->crtc_h = ids[i];
        else if (!strcmp(nm, "zpos") && !(p.flags & DRM_MODE_PROP_IMMUTABLE) && (p.flags & DRM_MODE_PROP_RANGE) &&
                 p.count_values == 2) {
            pp->zpos = ids[i];
            pp->zpos_max = range[1];
        } else if (!strcmp(nm, "type") && type)
            *type = (int)vals[i];
    }
    return (pp->fb_id && pp->crtc_id && pp->src_w && pp->src_h && pp->crtc_w && pp->crtc_h) ? 0 : -ENOENT;
}

/* the primary plane of our CRTC (the one showing it, else any primary it may use) and a free overlay plane
 * that can do ARGB8888 for the OSD (the highest id, which the kernel stacks on top when zpos ties) */
static int find_planes(uint32_t *primary, uint32_t *osd) {
    uint32_t planes[32];
    struct drm_mode_get_plane_res pr;
    memset(&pr, 0, sizeof pr);
    pr.plane_id_ptr = (uint64_t)(uintptr_t)planes;
    pr.count_planes = 32;
    int e = xioctl(T.fd, DRM_IOCTL_MODE_GETPLANERESOURCES, &pr);
    if (e)
        return e;
    uint32_t n = pr.count_planes < 32 ? pr.count_planes : 32, prim_any = 0;
    *primary = *osd = 0;
    for (uint32_t i = 0; i < n; i++) {
        uint32_t fmts[64];
        struct drm_mode_get_plane p;
        memset(&p, 0, sizeof p);
        p.plane_id = planes[i];
        p.format_type_ptr = (uint64_t)(uintptr_t)fmts;
        p.count_format_types = 64;
        if (xioctl(T.fd, DRM_IOCTL_MODE_GETPLANE, &p) || !(p.possible_crtcs & (1u << T.crtc_index)))
            continue;
        plane_props pp;
        int type = -1;
        if (plane_props_get(p.plane_id, &pp, &type))
            continue;
        if (type == 1) {                                   /* DRM_PLANE_TYPE_PRIMARY */
            if (p.crtc_id == T.crtc_id)
                *primary = p.plane_id;
            else if (!prim_any && !p.crtc_id)
                prim_any = p.plane_id;
        } else if (type == 0 && !p.crtc_id && !p.fb_id) { /* DRM_PLANE_TYPE_OVERLAY, free */
            uint32_t nf = p.count_format_types < 64 ? p.count_format_types : 64;
            for (uint32_t f = 0; f < nf; f++)
                if (fmts[f] == TB_ARGB8888 && p.plane_id > *osd)
                    *osd = p.plane_id;
        }
    }
    if (!*primary)
        *primary = prim_any;
    return *primary ? 0 : -ENOENT;
}

typedef struct {
    uint32_t objs[4], counts[4], props[48];
    uint64_t vals[48];
    int nobj, nprop;
} areq;

static void a_obj(areq *r, uint32_t obj) {
    r->objs[r->nobj] = obj;
    r->counts[r->nobj] = 0;
    r->nobj++;
}

static void a_prop(areq *r, uint32_t prop, uint64_t v) {
    if (!prop || r->nprop >= 48 || !r->nobj)
        return;
    r->props[r->nprop] = prop;
    r->vals[r->nprop] = v;
    r->nprop++;
    r->counts[r->nobj - 1]++;
}

static void plane_set(areq *r, const plane_props *pp, uint32_t plane, uint32_t fb, uint32_t sw, uint32_t sh,
                      uint32_t dw, uint32_t dh, int64_t zpos) {
    a_obj(r, plane);
    a_prop(r, pp->fb_id, fb);
    a_prop(r, pp->crtc_id, fb ? T.crtc_id : 0);
    a_prop(r, pp->src_x, 0);
    a_prop(r, pp->src_y, 0);
    a_prop(r, pp->src_w, (uint64_t)sw << 16);
    a_prop(r, pp->src_h, (uint64_t)sh << 16);
    a_prop(r, pp->crtc_x, 0);
    a_prop(r, pp->crtc_y, 0);
    a_prop(r, pp->crtc_w, dw);
    a_prop(r, pp->crtc_h, dh);
    if (zpos >= 0 && pp->zpos)
        a_prop(r, pp->zpos, (uint64_t)zpos);
}

static int a_commit(areq *r, uint32_t flags) {
#ifdef TB_TEST_FAKE
    (void)r;
    if (!(flags & DRM_MODE_ATOMIC_TEST_ONLY))
        fake_flip_at = now_ms() + 2.0;
    return 0;
#endif
    struct drm_mode_atomic a;
    memset(&a, 0, sizeof a);
    a.flags = flags;
    a.count_objs = (uint32_t)r->nobj;
    a.objs_ptr = (uint64_t)(uintptr_t)r->objs;
    a.count_props_ptr = (uint64_t)(uintptr_t)r->counts;
    a.props_ptr = (uint64_t)(uintptr_t)r->props;
    a.prop_values_ptr = (uint64_t)(uintptr_t)r->vals;
    a.user_data = 0x57e2e0;
    return xioctl(T.fd, DRM_IOCTL_MODE_ATOMIC, &a);
}

/* a flip completed: the buffers of that commit are on screen, the ones they replaced are free again */
static void flip_done(uint32_t seq, double when_ms) {
    pthread_mutex_lock(&C.mu);
    for (int li = 0; li < 2; li++) {
        if (C.flying[li] >= 0)
            C.shown[li] = C.flying[li];
        C.flying[li] = -1;
    }
    if (!C.osd_enabled)
        C.shown[1] = -1;
    C.flip_pending = 0;
    C.completed++;
    C.last_seq = seq;
    C.last_flip_ms = when_ms;
    if (C.retired.n && C.completed >= C.retire_after)
        layer_free(&C.retired);                  /* nothing of the old size is on screen any more */
    pthread_cond_broadcast(&C.cv);
    pthread_mutex_unlock(&C.mu);
}

static int wait_flip(int timeout_ms) {
#ifdef TB_TEST_FAKE
    (void)timeout_ms;
    if (C.flip_pending) {
        double d = fake_flip_at - now_ms();
        if (d > 0)
            sleep_us((long)(d * 1000));
        flip_done(0, now_ms());
    }
    return 0;
#endif
    double t0 = now_ms();
    while (C.flip_pending) {
        int left = timeout_ms - (int)(now_ms() - t0);
        if (left <= 0)
            return -ETIMEDOUT;
        struct pollfd pfd = { .fd = T.fd, .events = POLLIN };
        int r = poll(&pfd, 1, left);
        if (r < 0) {
            if (errno == EINTR)
                continue;
            return -errno;
        }
        if (r == 0)
            return -ETIMEDOUT;
        char buf[1024];
        ssize_t n = read(T.fd, buf, sizeof buf);
        if (n < 0) {
            if (errno == EINTR || errno == EAGAIN)
                continue;
            return -errno;
        }
        for (ssize_t off = 0; off + (ssize_t)sizeof(struct drm_event) <= n;) {
            struct drm_event *ev = (struct drm_event *)(buf + off);
            if (ev->length < sizeof *ev || off + (ssize_t)ev->length > n)
                break;
            if (ev->type == DRM_EVENT_FLIP_COMPLETE && ev->length >= sizeof(struct drm_event_vblank)) {
                struct drm_event_vblank *vb = (struct drm_event_vblank *)ev;
                flip_done(vb->sequence, vb->tv_sec * 1000.0 + vb->tv_usec / 1000.0);
            }
            off += ev->length;
        }
    }
    return 0;
}

/* A frame at a lower rate than the display's (a mode at 30 fps on a 60 Hz display: interval 2) is committed no
 * earlier than vblank last_seq + n - 1, so it lands n vblanks after the previous flip and stays up for exactly
 * n refreshes: the rate is locked to the display (a timer that sleeps to 30 fps drifts against the vblanks and
 * shows frames for 1, 2 or 3 refreshes - stock's judder). The previous flip must be done (comp_do waited) */
static int wait_interval(int n) {
    pthread_mutex_lock(&C.mu);
    const uint64_t done = C.completed;
    const uint32_t seq = C.last_seq;
    const double at = C.last_flip_ms;
    pthread_mutex_unlock(&C.mu);
    if (n <= 1 || done == 0)
        return 0;
#ifdef TB_TEST_FAKE
    (void)seq;
    double d = at + (n - 1) * FAKE_VBL_MS - now_ms();
    if (d > 0)
        sleep_us((long)(d * 1000.0));
    return 0;
#else
    (void)at;
    uint32_t now = 0, target = seq + (uint32_t)(n - 1);
    int e = vbl_now(&now);
    if (e)
        return e;
    if ((int32_t)(now - target) < 0)
        e = vbl_wait_until(target);
    return e;
#endif
}

/* one job: copy + clear the frame, copy the OSD, commit both (interval: see wait_interval) */
static int comp_do(uint8_t *src, int pitch, int w, int h, int clear, uint32_t cx,
                   const uint8_t *osrc, int opitch, int ow, int oh, int oshow, int interval) {
    double t0 = now_ms();
    if ((uint32_t)w != C.lay[0].w || (uint32_t)h != C.lay[0].h) {        /* render size changed */
        if (C.retired.n) {                                                /* an older set is still pending */
            int e = wait_flip(200);
            if (e)
                return e;
            if (C.retired.n) {
                pthread_mutex_lock(&C.mu);
                layer_free(&C.retired);
                pthread_mutex_unlock(&C.mu);
            }
        }
        layer fresh;
        int e = layer_alloc(&fresh, (uint32_t)w, (uint32_t)h, 24);
        if (e)
            return e;
        pthread_mutex_lock(&C.mu);
        if (C.lay[0].n) {
            C.retired = C.lay[0];                        /* freed after the next commit is on screen */
            C.retire_after = C.issued + 1;   /* = once the first commit using the new set has landed */
        }
        C.lay[0] = fresh;
        C.shown[0] = C.flying[0] = -1;                   /* the old indices meant the old set */
        pthread_mutex_unlock(&C.mu);
    }
    int b = free_index(0);
    if (b < 0) {
        int e = wait_flip(200);
        if (e)
            return e;
        b = free_index(0);
        if (b < 0)
            return -EDEADLK;
    }
#ifdef TB_TEST_FAKE
    __atomic_store_n(&fake_copying_layer0, b, __ATOMIC_SEQ_CST);
    if (b == __atomic_load_n(&C.shown[0], __ATOMIC_SEQ_CST) || b == __atomic_load_n(&C.flying[0], __ATOMIC_SEQ_CST))
        __atomic_add_fetch(&fake_comp_violations, 1, __ATOMIC_RELAXED);
#endif
    tb_copy_rows(C.lay[0].buf[b].map, (int)C.lay[0].buf[b].pitch, src, pitch, w * 4, h);
#ifdef TB_TEST_FAKE
    __atomic_store_n(&fake_copying_layer0, -1, __ATOMIC_SEQ_CST);
#endif
    int osd_on = oshow && osrc && C.osd && C.lay[1].n && ow == (int)C.lay[1].w && oh == (int)C.lay[1].h;
    preview_grab(src, pitch, w, h, osd_on ? osrc : NULL, opitch, ow, oh);   /* the browser stream, only while
                                                                              someone watches; with the OSD */
    double t1 = now_ms();
    if (clear)
        fill32(src, pitch, w, h, cx);
    double t2 = now_ms();
    pthread_mutex_lock(&C.mu);
    C.src_busy = 0;                                      /* the engine may draw into that surface again */
    pthread_cond_broadcast(&C.cv);
    pthread_mutex_unlock(&C.mu);
    int ob = -1;
    if (osd_on) {
        ob = free_index(1);
        if (ob < 0) {
            int e = wait_flip(200);
            if (e)
                return e;
            ob = free_index(1);
        }
        if (ob >= 0)
            tb_copy_rows(C.lay[1].buf[ob].map, (int)C.lay[1].buf[ob].pitch, osrc, opitch, ow * 4, oh);
    }
    double t3 = now_ms();
    if (C.flip_pending) {
        int e = wait_flip(200);
        if (e)
            return e;
    }
    {
        int e = wait_interval(interval);
        if (e)
            return e;
    }
#ifdef TB_TEST_FAKE
    if (C.completed) {
        double gap = now_ms() - C.last_flip_ms;
        fake_commits_at[interval]++;
        if (gap < fake_min_gap_ms[interval])
            fake_min_gap_ms[interval] = gap;
    }
#endif
    double t4 = now_ms();
    areq r;
    memset(&r, 0, sizeof r);
    plane_set(&r, &C.pp[0], C.primary, C.lay[0].buf[b].fb_id, (uint32_t)w, (uint32_t)h, T.width, T.height, 0);
    int osd_now = ob >= 0;
    if (C.osd && (osd_now || C.osd_enabled)) {
        if (osd_now)
            plane_set(&r, &C.pp[1], C.osd, C.lay[1].buf[ob].fb_id, (uint32_t)ow, (uint32_t)oh, (uint32_t)ow,
                      (uint32_t)oh, (int64_t)C.pp[1].zpos_max);
        else
            plane_set(&r, &C.pp[1], C.osd, 0, C.lay[1].w, C.lay[1].h, C.lay[1].w, C.lay[1].h, -1);
    }
    int e = a_commit(&r, DRM_MODE_ATOMIC_NONBLOCK | DRM_MODE_PAGE_FLIP_EVENT);
    if (e == -EBUSY) {                                   /* belt and braces: we waited for the last flip */
        sleep_us(500);
        e = a_commit(&r, DRM_MODE_ATOMIC_NONBLOCK | DRM_MODE_PAGE_FLIP_EVENT);
    }
    double t5 = now_ms();
    if (e)
        return e;
    pthread_mutex_lock(&C.mu);
    C.flying[0] = b;
    C.flying[1] = ob;
    C.osd_enabled = osd_now;
    C.flip_pending = 1;
    C.issued++;
    C.last_commit_ms = t4;
    C.copy_ms += t1 - t0;
    C.clear_ms += t2 - t1;
    C.osd_ms += t3 - t2;
    C.flipwait_ms += t4 - t3;
    C.commit_ms += t5 - t4;
    pthread_mutex_unlock(&C.mu);
    return 0;
}

static void *comp_worker(void *arg) {
    (void)arg;
    pthread_mutex_lock(&C.mu);
    for (;;) {
        while (!C.stop && !C.has_job)
            pthread_cond_wait(&C.cv, &C.mu);
        if (C.stop)
            break;
        uint8_t *src = C.src;
        int pitch = C.pitch, w = C.w, h = C.h, clear = C.clear;
        uint32_t cx = C.clear_xrgb;
        const uint8_t *osrc = C.osd_src;
        int op = C.osd_pitch, ow = C.osd_w, oh = C.osd_h, oshow = C.osd_show, iv = C.interval;
        C.has_job = 0;
        C.job_busy = 1;
        C.src_busy = 1;
        pthread_cond_broadcast(&C.cv);
        pthread_mutex_unlock(&C.mu);
        int e = comp_do(src, pitch, w, h, clear, cx, osrc, op, ow, oh, oshow, iv);
        pthread_mutex_lock(&C.mu);
        C.job_busy = 0;
        C.src_busy = 0;
        if (e) {
            C.errors++;
            C.last_err = e;
        } else {
            C.frames++;
        }
        pthread_cond_broadcast(&C.cv);
    }
    pthread_mutex_unlock(&C.mu);
    return NULL;
}

/* after tb_modeset. osd_w/osd_h = size of the OSD overlay plane (0, 0 = none). 0 or a negative code. */
int tb_comp_init(int osd_w, int osd_h) {
#ifdef TB_TEST_FAKE
    T.ready = 1;
    if (!T.width) {
        T.width = 1280;
        T.height = 720;
    }
#endif
    if (!T.ready)
        return -ENODEV;
    if (C.ready)
        return -EALREADY;
    if (osd_w < 0 || osd_h < 0 || osd_w > (int)T.width || osd_h > (int)T.height)
        return -EINVAL;
#ifdef TB_TEST_FAKE
    C.primary = 1;
    C.osd = osd_w ? 2 : 0;
    memset(C.pp, 0, sizeof C.pp);
#else
    struct drm_set_client_cap cap = { .capability = DRM_CLIENT_CAP_UNIVERSAL_PLANES, .value = 1 };
    int e = xioctl(T.fd, DRM_IOCTL_SET_CLIENT_CAP, &cap);
    if (e)
        return e;
    cap.capability = DRM_CLIENT_CAP_ATOMIC;
    if ((e = xioctl(T.fd, DRM_IOCTL_SET_CLIENT_CAP, &cap)))
        return e;
    if ((e = find_planes(&C.primary, &C.osd)))
        return e;
    if ((e = plane_props_get(C.primary, &C.pp[0], NULL)))
        return e;
    if (C.osd && plane_props_get(C.osd, &C.pp[1], NULL))
        C.osd = 0;
    if (!osd_w || !osd_h)
        C.osd = 0;
    /* prove the request shape on the current picture before switching anything */
    areq r;
    memset(&r, 0, sizeof r);
    plane_set(&r, &C.pp[0], C.primary, T.buf[T.front >= 0 ? T.front : 0].fb_id, T.width, T.height, T.width, T.height, 0);
    if ((e = a_commit(&r, DRM_MODE_ATOMIC_TEST_ONLY)))
        return e;
#endif
    if (C.osd) {
        int e2 = layer_alloc(&C.lay[1], (uint32_t)osd_w, (uint32_t)osd_h, 32);
        if (e2)
            C.osd = 0;
    }
    C.stop = 0;
    if (pthread_create(&C.th, NULL, comp_worker, NULL))
        return -EAGAIN;
    C.started = 1;
    C.ready = 1;
    return (int)C.primary;
}

int tb_comp_present(uint8_t *src, int pitch, int w, int h, int clear, uint32_t clear_xrgb,
                    const uint8_t *osd, int osd_pitch, int osd_w, int osd_h, int osd_show, int interval) {
    if (!C.ready)
        return -ENODEV;
    if (!src || ((uintptr_t)src & 3) || w < 16 || h < 16 || w > (int)T.width || h > (int)T.height || pitch < w * 4)
        return -EINVAL;
    if (osd_show && (!osd || ((uintptr_t)osd & 3) || osd_pitch < osd_w * 4 || osd_w < 1 || osd_h < 1))
        return -EINVAL;
    double t0 = now_ms();
    pthread_mutex_lock(&C.mu);
    while ((C.has_job || C.job_busy) && !C.stop)
        pthread_cond_wait(&C.cv, &C.mu);
    int err = C.last_err;
    if (!err && !C.stop) {
        C.src = src;
        C.pitch = pitch;
        C.w = w;
        C.h = h;
        C.clear = clear;
        C.clear_xrgb = clear_xrgb;
        C.osd_src = osd;
        C.osd_pitch = osd_pitch;
        C.osd_w = osd_w;
        C.osd_h = osd_h;
        C.osd_show = osd_show;
        C.interval = interval < 1 ? 1 : (interval > 4 ? 4 : interval);
        C.has_job = 1;
        C.src_busy = 1;
        C.submit_wait_ms += now_ms() - t0;
        pthread_cond_broadcast(&C.cv);
    }
    pthread_mutex_unlock(&C.mu);
    return err;
}

int tb_comp_sync(void) {
    if (!C.ready)
        return -ENODEV;
    pthread_mutex_lock(&C.mu);
    while ((C.has_job || C.src_busy) && !C.stop)
        pthread_cond_wait(&C.cv, &C.mu);
    int err = C.last_err;
    pthread_mutex_unlock(&C.mu);
    return err;
}

/* copy what the mode layer shows right now into dst (w x h must be the current render size); for screen grabs */
int tb_comp_snapshot(uint8_t *dst, int pitch, int w, int h) {
    if (!C.ready)
        return -ENODEV;
    if (!dst || pitch < w * 4)
        return -EINVAL;
    pthread_mutex_lock(&C.mu);
    int b = C.shown[0] >= 0 ? C.shown[0] : C.flying[0];
    int e = 0;
    if (b < 0 || (uint32_t)w != C.lay[0].w || (uint32_t)h != C.lay[0].h)
        e = -ENOENT;
    else
        tb_copy_rows(dst, pitch, C.lay[0].buf[b].map, (int)C.lay[0].buf[b].pitch, w * 4, h);
    pthread_mutex_unlock(&C.mu);
    return e;
}

/* wait until the worker has nothing queued or in hand: the frame handed over last is committed (it goes on screen
 * at the next vblank) and the next tb_comp_present() will not wait. The engine calls this before it reads the audio
 * for a frame, when the frame fits in a refresh: the frame then reaches the screen one refresh sooner after its
 * audio was read (main.py, "late start") */
int tb_comp_wait_idle(void) {
    if (!C.ready)
        return -ENODEV;
    pthread_mutex_lock(&C.mu);
    while ((C.has_job || C.job_busy) && !C.stop)
        pthread_cond_wait(&C.cv, &C.mu);
    int err = C.last_err;
    pthread_mutex_unlock(&C.mu);
    return err;
}

/* stop the worker (after the frame in flight lands); the display keeps the last picture */
int tb_comp_stop(void) {
    if (!C.ready)
        return -ENODEV;
    pthread_mutex_lock(&C.mu);
    while ((C.has_job || C.job_busy) && !C.stop)
        pthread_cond_wait(&C.cv, &C.mu);
    C.stop = 1;
    pthread_cond_broadcast(&C.cv);
    pthread_mutex_unlock(&C.mu);
    if (C.started)
        pthread_join(C.th, NULL);
    C.started = 0;
    int e = wait_flip(200);
    C.ready = 0;
    return e;
}

/* frames, issued, completed, errors, copy_ms, clear_ms, osd_ms, flipwait_ms, commit_ms, submit_wait_ms,
 * last_err, last vblank seq, layer w, layer h, primary plane, osd plane, last flip time (ms, CLOCK_MONOTONIC),
 * last commit asked for (ms, CLOCK_MONOTONIC; v7) */
int tb_comp_stats(double *out, int n) {
    pthread_mutex_lock(&C.mu);
    double v[] = { (double)C.frames, (double)C.issued, (double)C.completed, (double)C.errors, C.copy_ms, C.clear_ms,
                   C.osd_ms, C.flipwait_ms, C.commit_ms, C.submit_wait_ms, (double)C.last_err, (double)C.last_seq,
                   (double)C.lay[0].w, (double)C.lay[0].h, (double)C.primary, (double)C.osd, C.last_flip_ms,
                   C.last_commit_ms };
    pthread_mutex_unlock(&C.mu);
    int k = (int)(sizeof v / sizeof v[0]);
    if (!out || n <= 0)
        return k;
    for (int i = 0; i < n && i < k; i++)
        out[i] = v[i];
    return k;
}
