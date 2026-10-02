"""display.py - stereopsis's display. It owns the screen through kms.c: the mode is set once (legacy KMS),
then the compositor (kms.c v4) presents every frame with one atomic commit from its own thread:
  * the mode layer = the frame at its render size; the display hardware scales it to full screen
  * the OSD layer  = an ARGB surface on an overlay plane on top (only while the OSD is up)
  * the copy into scanout memory and the clear for the next frame happen off the engine's thread.
pygame runs on SDL's "dummy" video driver: surfaces, fonts, image loading and convert() work as before,
SDL just never touches the screen. If the compositor cannot start, frames go the 0.1 way (legacy page
flips, full size only).

kms.c is compiled on the device with gcc the first time (a few seconds) and cached by a hash of its
source and flags in build/ next to this file (or /tmp if that is not writable).
Set STEREOPSIS_NULL_DISPLAY=1 to run without a display at all (tests on a PC).
"""
import ctypes
import hashlib
import os
import platform
import subprocess
import time

HERE = os.path.dirname(os.path.abspath(__file__))
KMS_VERSION = 7
FLAGS = ["-O2", "-D_FILE_OFFSET_BITS=64", "-Wall", "-shared", "-fPIC", "-pthread"]
LIBS = ["-ljpeg"]                        # the browser preview (libjpeg-turbo is on the EYESY image)
# the EYESY is a Cortex-A53 running 32-bit code: without these, gcc targets an older ARM with no hardware
# divide and no NEON (a division per pixel in the preview downscaler once cost 75 ms a frame)
ARCH = ["-mcpu=cortex-a53", "-mfpu=neon-fp-armv8", "-mfloat-abi=hard"] if platform.machine().startswith("armv7") else []
PREVIEW_W, PREVIEW_H, PREVIEW_Q, PREVIEW_EVERY_MS = 640, 360, 70, 66
OSD_W, OSD_H = 640, 480                  # the stock OSD draws inside this corner


def _log(msg):
    print("[stereopsis] " + msg, flush=True)


def build():
    """Compile (or reuse) kms.c and load it. Returns the ctypes library."""
    src_path = os.path.join(HERE, "kms.c")
    src = open(src_path, "rb").read()
    mach = platform.machine() or "unknown"
    tag = hashlib.sha1(src + " ".join(FLAGS + ARCH + LIBS).encode()).hexdigest()[:10]
    dirs = [os.path.join(HERE, "build"), "/tmp"]
    so = None
    for d in dirs:
        cand = os.path.join(d, "kms_%s_%s.so" % (tag, mach))
        if os.path.exists(cand):
            so = cand
            break
    if so is None:
        t0 = time.time()
        for d in dirs:
            try:
                os.makedirs(d, exist_ok=True)
            except Exception:
                continue
            cand = os.path.join(d, "kms_%s_%s.so" % (tag, mach))
            tmp = cand + ".tmp%d" % os.getpid()
            r = subprocess.run(["gcc"] + FLAGS + ARCH + ["-o", tmp, src_path] + LIBS, capture_output=True, text=True, timeout=240)
            if r.returncode != 0:                        # no libjpeg: build without the browser preview
                _log("gcc with %s failed (%s): building without the preview" % (" ".join(LIBS), r.stderr.strip()[-200:]))
                r = subprocess.run(["gcc"] + FLAGS + ARCH + ["-DTB_NO_JPEG", "-o", tmp, src_path], capture_output=True,
                                   text=True, timeout=240)
            if r.returncode == 0:
                os.replace(tmp, cand)
                so = cand
                _log("compiled %s in %.1f s%s" % (os.path.basename(cand), time.time() - t0,
                                                  (" (warnings: %s)" % r.stderr.strip()[-300:]) if r.stderr.strip() else ""))
                break
            _log("gcc failed in %s: %s" % (d, r.stderr.strip()[-600:]))
        if so is None:
            raise RuntimeError("kms.c did not compile")
        for d in dirs:                                   # drop builds of older kms.c versions
            try:
                for f in os.listdir(d):
                    if f.startswith("kms_") and f.endswith(".so") and os.path.join(d, f) != so:
                        os.remove(os.path.join(d, f))
            except Exception:
                pass
    lib = ctypes.CDLL(so)
    lib.tb_version.restype = ctypes.c_int
    lib.tb_open.argtypes = [ctypes.c_char_p]
    lib.tb_modeset.argtypes = [ctypes.c_int] * 5
    lib.tb_probe.argtypes = [ctypes.c_int, ctypes.c_char_p, ctypes.c_int]
    lib.tb_present.argtypes = [ctypes.c_void_p, ctypes.c_int, ctypes.c_int, ctypes.c_int]
    lib.tb_stats.argtypes = [ctypes.POINTER(ctypes.c_double), ctypes.c_int]
    lib.tb_comp_init.argtypes = [ctypes.c_int, ctypes.c_int]
    lib.tb_comp_present.argtypes = [ctypes.c_void_p, ctypes.c_int, ctypes.c_int, ctypes.c_int, ctypes.c_int,
                                    ctypes.c_uint32, ctypes.c_void_p, ctypes.c_int, ctypes.c_int, ctypes.c_int,
                                    ctypes.c_int, ctypes.c_int]
    lib.tb_comp_snapshot.argtypes = [ctypes.c_void_p, ctypes.c_int, ctypes.c_int, ctypes.c_int]
    lib.tb_comp_stats.argtypes = [ctypes.POINTER(ctypes.c_double), ctypes.c_int]
    lib.tb_preview_init.argtypes = [ctypes.c_int] * 4
    lib.tb_preview_jpeg.argtypes = [ctypes.c_void_p, ctypes.c_int, ctypes.POINTER(ctypes.c_uint64),
                                    ctypes.POINTER(ctypes.c_int)]
    lib.tb_preview_stats.argtypes = [ctypes.POINTER(ctypes.c_double), ctypes.c_int]
    if lib.tb_version() != KMS_VERSION:
        raise RuntimeError("kms.c version %d, expected %d" % (lib.tb_version(), KMS_VERSION))
    return lib


def xrgb(color):
    """a pygame-style colour (floats allowed, like stock bg colours) as the XRGB8888 word pygame's fill writes"""
    r, g, b = (max(0, min(255, int(c))) for c in tuple(color)[:3])
    return (r << 16) | (g << 8) | b


class Display(object):
    """present(frame, clear=bg or None, osd=surface or None) puts a finished frame on screen and returns as soon
    as the compositor has taken it (it waits only while the previous frame is still being handed over)."""

    def __init__(self):
        self.lib = None
        self.fd = -1
        self.w = self.h = self.hz = 0
        self.null = os.environ.get("STEREOPSIS_NULL_DISPLAY") == "1"
        self.comp = False                        # the v4 compositor runs (else legacy full-size flips)
        self.osd_plane = False
        self.presents = 0
        self.preview = False                     # the compositor makes browser preview JPEGs
        self._null_small = None                  # STEREOPSIS_NULL_DISPLAY: a small copy for the preview
        self._null_seq = 0
        self._null_at = 0.0

    def open(self, want_w, want_h, want_hz=60):
        if self.null:
            self.w, self.h, self.hz = want_w, want_h, want_hz
            self.comp, self.osd_plane, self.preview = True, True, True   # tests exercise these code paths
            _log("display: NULL (STEREOPSIS_NULL_DISPLAY=1) %dx%d" % (want_w, want_h))
            return
        self.lib = build()
        fd = -1
        for node in ("/dev/dri/card0", "/dev/dri/card1"):
            fd = self.lib.tb_open(node.encode())
            if fd >= 0:
                break
        if fd < 0:
            raise RuntimeError("no DRM device we can drive (tb_open %d)" % fd)
        rc = self.lib.tb_modeset(fd, want_w, want_h, want_hz, 3)
        if rc != 0:
            raise RuntimeError("tb_modeset %dx%d@%d failed: %d" % (want_w, want_h, want_hz, rc))
        self.fd = fd
        st = self.stats()
        self.w, self.h, self.hz = int(st[11]), int(st[12]), int(st[15])
        _log("display: KMS %dx%d@%d on crtc %d (index %d), scanout pitch %d"
             % (self.w, self.h, self.hz, st[9], st[10], st[13]))
        rc = self.lib.tb_comp_init(OSD_W, OSD_H)
        if rc > 0:
            self.comp = True
            cs = self.comp_stats()
            self.osd_plane = cs[15] > 0
            _log("compositor: atomic commits, primary plane %d, OSD plane %s"
                 % (rc, int(cs[15]) if self.osd_plane else "none"))
        else:
            _log("compositor unavailable (%d): full-size legacy page flips instead" % rc)
        if self.comp:
            rc = self.lib.tb_preview_init(PREVIEW_W, PREVIEW_H, PREVIEW_Q, PREVIEW_EVERY_MS)
            self.preview = rc == 0
            _log("browser preview: %s" % ("%dx%d JPEG, up to %d fps while someone watches"
                                            % (PREVIEW_W, PREVIEW_H, 1000 // PREVIEW_EVERY_MS) if self.preview else "off (%d)" % rc))

    def present(self, frame, clear=None, osd=None, interval=1):
        """frame: XRGB8888 surface up to the display's size. clear: a colour to clear frame to once copied
        (None = keep it, persist). osd: an ARGB surface of OSD_W x OSD_H, or None = no OSD. interval: the
        frame's swap interval (the compositor shows it that many vblanks after the previous one: 2 = 30 fps
        on a 60 Hz display, locked to it)."""
        self.presents += 1
        if self.null:
            now = time.time()
            if now - self._null_at >= PREVIEW_EVERY_MS / 1000.0:     # tests: a small copy for the preview
                import pygame
                self._null_small = pygame.transform.scale(frame, (PREVIEW_W, PREVIEW_H))
                self._null_seq += 1
                self._null_at = now
            if clear is not None:
                frame.fill(tuple(clear)[:3])            # the compositor would clear it off-thread
            return 0
        if not self.comp:
            return self.lib.tb_present(frame._pixels_address, frame.get_pitch(), frame.get_width(), frame.get_height())
        show = osd is not None and self.osd_plane
        return self.lib.tb_comp_present(frame._pixels_address, frame.get_pitch(), frame.get_width(), frame.get_height(),
                                        1 if clear is not None else 0, xrgb(clear) if clear is not None else 0,
                                        osd._pixels_address if show else None, osd.get_pitch() if show else 0,
                                        osd.get_width() if show else 0, osd.get_height() if show else 0,
                                        1 if show else 0, int(interval))

    def sync(self):
        """wait until the compositor no longer reads or writes the last presented frame"""
        if self.comp and not self.null:
            return self.lib.tb_comp_sync()
        return 0

    def snapshot(self, surf):
        """copy what is on screen now into surf (the current render size); slow (reads scanout memory)"""
        if self.null or not self.comp:
            return -1
        self.lib.tb_comp_sync()
        return self.lib.tb_comp_snapshot(surf._pixels_address, surf.get_pitch(), surf.get_width(), surf.get_height())

    def wait_idle(self):
        """wait until the compositor has committed the frame handed over last (the next present() will not wait)"""
        if self.comp and not self.null:
            return self.lib.tb_comp_wait_idle()
        return 0

    def close(self):
        """stop the preview and compositor threads (the frame in flight lands first; the last picture stays on
        screen until the process exits and the kernel takes the display back)"""
        if self.null or self.lib is None:
            return
        if self.preview:
            self.lib.tb_preview_stop()
            self.preview = False
        if self.comp:
            self.lib.tb_comp_stop()
            self.comp = False

    def preview_jpeg(self, buf, seq):
        """the newest preview JPEG if newer than seq: (length, new_seq); length 0 = nothing newer yet.
        buf is a ctypes char buffer owned by the caller (one per viewer). The compositor only makes
        previews while this keeps being called."""
        if self.null:
            if self._null_small is None or self._null_seq == seq:
                return 0, seq
            import io
            import pygame
            f = io.BytesIO()
            pygame.image.save(self._null_small, f, "JPEG")
            data = f.getvalue()
            ctypes.memmove(buf, data, min(len(data), len(buf)))
            return min(len(data), len(buf)), self._null_seq
        if not self.preview:
            return -1, seq
        s = ctypes.c_uint64(seq)
        need = ctypes.c_int(0)
        n = self.lib.tb_preview_jpeg(buf, len(buf), ctypes.byref(s), ctypes.byref(need))
        return n, s.value

    def preview_stats(self):
        out = (ctypes.c_double * 9)()
        if self.preview and self.lib is not None and not self.null:
            self.lib.tb_preview_stats(out, 9)
        return list(out)

    def stats(self):
        out = (ctypes.c_double * 16)()
        if self.lib is not None:
            self.lib.tb_stats(out, 16)
        return list(out)

    def comp_stats(self):
        out = (ctypes.c_double * 18)()
        if self.lib is not None and not self.null:
            self.lib.tb_comp_stats(out, 18)
        return list(out)

    def probe(self):
        if self.lib is None or self.fd < 0:
            return "no display"
        buf = ctypes.create_string_buffer(16384)
        self.lib.tb_probe(self.fd, buf, 16384)
        return buf.value.decode("utf-8", "replace")
