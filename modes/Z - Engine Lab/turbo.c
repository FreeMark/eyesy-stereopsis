/* turbo.c - "Z - Engine Lab": Phase 1 of the EYESY engine fork (stereopsis).
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
 */
#define _GNU_SOURCE
#include <errno.h>
#include <pthread.h>
#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <time.h>
#include <drm/drm.h>
#include <drm/drm_mode.h>

#define TB_VERSION 2
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
    uint32_t crtc_id, crtc_index, conn_id, width, height, orig_fb;
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

static int make_buf(int fd, uint32_t w, uint32_t h, tb_buf *b) {
    memset(b, 0, sizeof *b);
#ifdef TB_TEST_FAKE
    (void)fd;
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
    f.depth = 24;                   /* legacy (32, 24) = XRGB8888, the same as SDL's framebuffers */
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
 * width, height, pitch, last vblank seq */
int tb_stats(double *out, int n) {
    double v[] = { (double)T.frames, (double)T.flips, (double)T.ebusy, (double)T.vblank_waits, (double)T.errors,
                   T.copy_ms, T.wait_ms, T.flip_ms, (double)T.last_err, (double)T.crtc_id, (double)T.crtc_index,
                   (double)T.width, (double)T.height, (double)(T.nbuf ? T.buf[0].pitch : 0), (double)T.last_seq };
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
