import os
import glob
import math
import time
import ctypes
import hashlib
import platform
import subprocess
import threading
import pygame

# 12 - Plasma Globe
#
# A plasma ball. Filaments of plasma run from the electrode to the glass, each one a band of the spectrum burning
# with it; the louder the music, the more of them burn. They bow and fizz (the treble makes them fizzier), fork near
# the glass and light a hot spot where they touch it, drifting over it, a little upward. Every kick makes them flare;
# the electrode glows with the bass.
#
#Knob1 - voltage: a few calm filaments on the left -> an electric storm on the right
#Knob2 - turn: the globe turns one way left of centre, the other right; centre holds still
#Knob3 - glow: bloom up to the middle; trails from 60 % up
#Knob4 - foreground color (the plasma; black gives the classic violet)
#Knob5 - background color
#Trigger - a hand on the glass: hold it and every filament gathers to where it touches, until you let go; a tap
#          (page, MIDI) touches for a moment
#
# Best on stereopsis (its engine analyses the audio: spectrum, bands, waveform, kicks). On the stock engine it still
# runs, on the level and the scope samples.
# The renderer is a C kernel (plasma.c + glow.h, next to this file) compiled ON the EYESY the first time the mode
# runs (~30 s, in the background, one build at a time; a simple preview shows meanwhile). Files: plasma_*.so =
# compiled cache (safe to delete); .native_trial / .native_strikes = crash guard; NATIVE_OFF (exists = never use the
# kernel; the crash guard writes it if the engine dies twice in a row while the kernel is on trial).
# Dev switches (files containing a number): DEBUG (1 = timings to the log), THREADS (2 = the final pass on two cores,
# the default; 1 = one), BENCH (1 or 2 = synthetic music instead of the input, for reproducible timings: 2 = a loud
# club mix), COUNT (how many (unused), default 1), KBENCH (exists = time the renderer's inner loops
# once, on the first frame, into the log), ASM (exists = also write the device's assembly of the kernel), DOWN (render
# at 1/DOWN of the surface's size; default: the full size on stereopsis, half above 400 lines on the stock engine).
# Written by tools/make_glow_modes.py from the template all glow.h modes share (edit that, not this file).

KERNEL = "plasma"
KERNEL_VERSION = 2
N_COMMON = 21                  # dt, knobs 1-5, fg rgb, bg rgb, level, bass, mid, treble, beat, kick, trig, prefilled, wave Hz
COUNT = 1
KNOWN_GOOD_FRAMES = 60
GUARD_SECONDS = 3.0
STEREOPSIS = os.environ.get("STEREOPSIS") == "1"     # the engine this runs on (stereopsis sets it)

S = {}                 # all state lives here (the module is re-imported on reload)


def _log(msg):
    print("[plasma] " + msg)


def _num(root, name, default):
    try:
        return float(open(os.path.join(root, name)).read().strip())
    except Exception:
        return default


# ---------------------------------------------------------------------------------------------- native build
def _native_allowed(root):
    """Crash guard. The marker is armed on the first native frame this mode actually DRAWS (not at load:
    every mode loads at boot) and cleared after KNOWN_GOOD_FRAMES, or GUARD_SECONDS after it was armed while
    the engine still runs (a mode shown for a moment, then the EYESY switched off, is not a crash). A marker
    left by a dead engine means it died while the kernel was on trial. Strike 1: the fallback for this session.
    Strike 2 in a row: NATIVE_OFF (permanent until deleted)."""
    if os.path.exists(os.path.join(root, "NATIVE_OFF")):
        return False, "NATIVE_OFF file present"
    marker = os.path.join(root, ".native_trial")
    if os.path.exists(marker):
        try:
            pid = int(open(marker).read().strip() or "0")
        except Exception:
            pid = 0
        if pid != os.getpid() and not os.path.exists("/proc/%d" % pid):
            strikes_f = os.path.join(root, ".native_strikes")
            try:
                strikes = int(open(strikes_f).read().strip() or "0") + 1
            except Exception:
                strikes = 1
            try:
                os.remove(marker)
                with open(strikes_f, "w") as f:
                    f.write(str(strikes))
                if strikes >= 2:
                    with open(os.path.join(root, "NATIVE_OFF"), "w") as f:
                        f.write("Auto-disabled %s: the video engine died twice in a row while the native renderer was on "
                                "trial.\nDelete this file to try the native renderer again.\n" % time.strftime("%Y-%m-%d %H:%M"))
            except Exception:
                pass
            if strikes >= 2:
                return False, "engine died twice during native trial - auto-disabled (see NATIVE_OFF)"
            return False, "engine died during native trial (strike 1 of 2) - fallback for this session"
    return True, ""


def _arm_guard():
    S["armed"] = True
    root = S["root"]
    try:
        with open(os.path.join(root, ".native_trial"), "w") as f:
            f.write(str(os.getpid()))
    except Exception:
        return

    def later():                                   # still alive GUARD_SECONDS on: no crash in the first frames
        time.sleep(GUARD_SECONDS)
        _clear_guard(root)
    threading.Thread(target=later, daemon=True).start()


def _clear_guard(root):
    for f in (".native_trial", ".native_strikes"):
        try:
            os.remove(os.path.join(root, f))
        except Exception:
            pass


def _find_or_build(root, sources, prefix):
    """The compiled kernel for this folder's sources: from this folder, /tmp, or another mode's folder (modes with the
    same sources share one build), else built here. One gcc at a time on the device (a lock in /tmp): gcc at -O3 needs
    ~100 MB and the EYESY has no swap, so every new mode compiling at once at the first boot could exhaust its memory.
    Returns the .so path, or None (why is logged)."""
    src_path = os.path.join(root, sources[0])
    src = b"".join(open(os.path.join(root, s), "rb").read() for s in sources)
    mach = platform.machine() or "unknown"
    base = ["-O3", "-fno-math-errno", "-shared", "-fPIC", "-pthread"]
    # the EYESY is a Cortex-A53 in 32-bit mode: without these, no hardware divide and no NEON
    arch = (["-mcpu=cortex-a53", "-mfpu=neon-fp-armv8", "-mfloat-abi=hard", "-funsafe-math-optimizations"]
            if mach.startswith("armv7") else [])
    tag = hashlib.sha1(src + " ".join(base + arch).encode()).hexdigest()[:10]
    name = "%s_%s_%s.so" % (prefix, tag, mach)
    parent = os.path.dirname(os.path.normpath(root))

    def look():
        for d in [root, "/tmp"] + sorted(glob.glob(os.path.join(glob.escape(parent), "*", ""))):
            p = os.path.join(d, name)
            if os.path.isfile(p):
                return p
        return None

    so = look()
    if so is None:
        lock = None
        try:
            import fcntl
            lock = open("/tmp/eyesy-kernel-build.lock", "a")
            fcntl.flock(lock, fcntl.LOCK_EX)
        except Exception:
            pass                                   # no fcntl (not Linux): build without the lock
        try:
            so = look()                            # built by another mode while this one waited
            if so is None:
                t0 = time.time()
                for d in (root, "/tmp"):
                    cand = os.path.join(d, name)
                    tmp = cand + ".tmp%d" % os.getpid()
                    for flags in (base + arch, base):
                        try:
                            r = subprocess.run(["gcc"] + flags + ["-o", tmp, src_path, "-lm"],
                                               capture_output=True, text=True, timeout=300)
                        except Exception as e:
                            _log("gcc not usable (%s) - fallback renderer" % e)
                            return None
                        if r.returncode == 0:
                            os.replace(tmp, cand)
                            so = cand
                            _log("compiled %s in %.1f s" % (name, time.time() - t0))
                            break
                        _log("gcc failed with %s: %s" % (" ".join(flags), r.stderr.strip()[-300:]))
                    if so:
                        break
        finally:
            if lock is not None:
                lock.close()                       # closing releases the lock
    if so is not None:
        for d in (root, "/tmp"):                   # drop builds of older sources (a loaded one stays mapped)
            try:
                for f in os.listdir(d):
                    if f.startswith(prefix + "_") and f.endswith(".so") and os.path.join(d, f) != so:
                        os.remove(os.path.join(d, f))
            except Exception:
                pass
    return so


def _compile_and_load(root):
    t0 = time.time()
    try:
        ok, why = _native_allowed(root)
        if not ok:
            _log("native renderer off: " + why)
            S["native_state"] = "off: " + why
            return
        so = _find_or_build(root, [KERNEL + ".c", "glow.h"], KERNEL)
        if so is None:
            S["native_state"] = "off: no kernel (see the log)"
            return
        lib = ctypes.CDLL(so)
        F = ctypes.POINTER(ctypes.c_float)
        lib.fx_init.argtypes = [ctypes.c_int, ctypes.c_int]
        lib.fx_frame.restype = ctypes.c_int
        lib.fx_frame.argtypes = [ctypes.c_void_p, ctypes.c_int, ctypes.c_int, ctypes.c_int, ctypes.c_int, ctypes.c_int,
                                 ctypes.c_int, ctypes.c_int, F, ctypes.c_int, ctypes.c_int, F, ctypes.c_int, ctypes.c_int,
                                 ctypes.c_void_p, ctypes.c_int, ctypes.c_float, ctypes.c_void_p, ctypes.c_int,
                                 ctypes.c_void_p, ctypes.c_int, F]
        lib.fx_stats.argtypes = [F, ctypes.c_int]
        if lib.fx_version() != KERNEL_VERSION or lib.fx_param_count() != N_COMMON + len(_extras(None, None)):
            S["native_state"] = "off: kernel version mismatch"
            return
        lib.fx_threads(int(_num(root, "THREADS", 2)))  # the final pass on two cores (THREADS 1: one, to compare)
        asm = os.path.join(root, KERNEL + ".s")
        if os.path.exists(os.path.join(root, "ASM")) and not os.path.exists(asm):   # dev: the device's code
            mach = platform.machine() or ""
            arch = (["-mcpu=cortex-a53", "-mfpu=neon-fp-armv8", "-mfloat-abi=hard", "-funsafe-math-optimizations"]
                    if mach.startswith("armv7") else [])
            subprocess.run(["gcc", "-O3", "-fno-math-errno", "-fPIC", "-pthread"] + arch +
                           ["-S", "-o", asm, os.path.join(root, KERNEL + ".c")], capture_output=True, timeout=300)
            _log("wrote %s.s" % KERNEL)
        S["lib"] = lib                             # fx_init runs on the engine's thread, at the first draw
        S["native_state"] = "on"
        _log("native renderer ready: %s (%.1f s)" % (os.path.basename(so), time.time() - t0))
    except Exception as e:
        S["native_state"] = "off: %s" % e
        _log("native load failed: %r" % (e,))


# ---------------------------------------------------------------------------------------------- setup
def setup(screen, eyesy):
    S.clear()
    root = eyesy.mode_root if getattr(eyesy, "mode_root", "") else os.path.dirname(os.path.abspath(__file__))
    S["root"] = root
    S["debug"] = _num(root, "DEBUG", 0) == 1
    S["count"] = int(_num(root, "COUNT", COUNT))
    S["down_file"] = _num(root, "DOWN", 0)
    S["P"] = (ctypes.c_float * (N_COMMON + len(_extras(None, None))))()
    S["zero"] = (ctypes.c_float * 2)()
    S["wave"] = (ctypes.c_float * 100)()
    S["bands"] = (ctypes.c_float * 32)()
    S["last"] = time.time()
    S["native_frames"] = 0
    S["native_state"] = "compiling"
    S["lvl"] = 0.0
    S["t"] = 0.0
    S["last_trig"] = -1.0
    S["prof"] = [0.0, 0, 0.0, 0]                       # DEBUG: seconds, frames, the longest, frames over 14.5 ms
    if os.path.exists(os.path.join(root, "BENCH")):
        _bench_setup(_num(root, "BENCH", 1) >= 2)
    threading.Thread(target=_compile_and_load, args=(root,), daemon=True).start()


def _surfaces(screen):
    """(Re)create the buffers for this draw surface (size / pixel format)."""
    xr, yr = int(screen.get_width()), int(screen.get_height())
    # the full size of the surface on stereopsis (it sets the surface's size: scale.json, the page); on the stock
    # engine (always 1280x720) half that, for speed. DOWN (a file) overrides
    down = int(S["down_file"]) if S["down_file"] >= 1 else (1 if yr <= 400 or STEREOPSIS else 2)
    down = max(1, min(8, down))
    key = (xr, yr, screen.get_bitsize(), screen.get_masks(), down)
    if S.get("surf_key") == key:
        return
    w, h = (xr + down - 1) // down, (yr + down - 1) // down
    bw, bh = (w + 3) // 4, (h + 3) // 4
    sh = screen.get_shifts()
    S.update(surf_key=key, xr=xr, yr=yr, w=w, h=h, bw=bw, bh=bh, down=down, shifts=(sh[0], sh[1], sh[2]))
    S["acc"] = (ctypes.c_float * (w * h * 3))()
    S["bloom"] = (ctypes.c_float * (bw * bh * 6))()
    S["native_fmt_ok"] = (screen.get_bytesize() == 4 and hasattr(screen, "_pixels_address")
                          and len({sh[0], sh[1], sh[2]}) == 3 and all(s in (0, 8, 16, 24) for s in sh[:3]))
    if not S["native_fmt_ok"]:
        _log("screen format not usable by the kernel (bytesize %d, shifts %s) - fallback" % (screen.get_bytesize(), sh))


def _bench_setup(heavy):
    """BENCH: 16 frames of a loud spectrum (bass-heavy, every band busy) and waveform, made once"""
    gain = 1.25 if heavy else 1.0
    ffts, waves = [], []
    for j in range(16):
        f = (ctypes.c_float * 1024)()
        w = (ctypes.c_float * 1024)()
        for k in range(1024):
            base = 0.9 * math.exp(-k / 60.0) + 0.4 * math.exp(-k / 400.0) + 0.12
            f[k] = min(1.0, gain * base * (0.75 + 0.25 * math.sin(j * 0.7 + k * 0.02)))
            w[k] = 0.8 * math.sin(k * 0.049 + j * 0.9) * (0.6 + 0.4 * math.sin(k * 0.011 + j))
        ffts.append(f)
        waves.append(w)
    S["bench"] = (ffts, waves, heavy)
    S["bench_ph"] = 0.0
    _log("BENCH %d: synthetic %s instead of the input" % (2 if heavy else 1, "club mix" if heavy else "loud music"))


def _bench_audio():
    ffts, waves, heavy = S["bench"]
    t = S["t"]
    ph = (t * 128.0 / 60.0) % 1.0
    kick = ph < S["bench_ph"]
    S["bench_ph"] = ph
    env = math.exp(-ph * 6.0)
    j = int(t * 30.0) % 16
    b = S["bands"]
    for i in range(32):
        b[i] = min(1.0, (0.85 - i * 0.015) * (0.7 + 0.3 * math.sin(t * 3.0 + i * 0.4)) + (0.3 * env if i < 6 else 0.0))
    if heavy:
        lv, bs, tr = 0.75 + 0.15 * math.sin(t * 2.1), 0.75 + 0.25 * env, 0.5 + 0.2 * math.sin(t * 7.3)
    else:
        lv, bs, tr = 0.55 + 0.2 * math.sin(t * 2.1), 0.4 + 0.55 * env, 0.35 + 0.2 * math.sin(t * 7.3)
    return (lv, bs, 0.45, tr, env, kick, ctypes.addressof(ffts[j]), 1024, 15.625, ctypes.addressof(waves[j]), 1024,
            32000.0, ctypes.addressof(b), 32)


def _audio(eyesy):
    """(level, bass, mid, treble, beat envelope, kick this frame, fft address, bins, bin Hz, wave address, samples,
    wave Hz, bands address, bands): stereopsis's analysis when it runs; on the stock engine, the level and the scope"""
    if S.get("bench"):
        return _bench_audio()
    if getattr(eyesy, "audio_analysis", False):
        fft, wave, b = eyesy.audio_fft, eyesy.audio_wave, S["bands"]
        src = eyesy.audio_bands
        for i in range(min(32, len(src))):
            b[i] = src[i]
        return (eyesy.audio_level, eyesy.audio_bass, eyesy.audio_mid, eyesy.audio_treble, eyesy.audio_beat,
                bool(eyesy.beat), ctypes.addressof(fft), len(fft), float(eyesy.audio_fft_hz),
                ctypes.addressof(wave), len(wave), 32000.0, ctypes.addressof(b), 32)
    ain = eyesy.audio_in
    peak = max(1, max(abs(int(v)) for v in ain)) / 32768.0
    S["lvl"] += (min(1.0, peak * 1.5) - S["lvl"]) * (0.5 if peak > S["lvl"] else 0.1)
    w = S["wave"]
    for i, v in enumerate(ain[:100]):
        w[i] = max(-1.0, min(1.0, v / 32768.0 * 2.0))
    lv = S["lvl"]
    return (lv, lv, lv * 0.6, lv * 0.3, 0.0, False, ctypes.addressof(S["zero"]), 0, 0.0,
            ctypes.addressof(w), len(ain[:100]), 2000.0, ctypes.addressof(S["zero"]), 0)


def _extras(eyesy, au):
    """the kernel's parameters after the common ones: 1 while the Trigger button is held (the hand on the glass)"""
    return [1.0 if eyesy is not None and getattr(eyesy, "key10_status", False) else 0.0]


# ---------------------------------------------------------------------------------------------- draw
def draw(screen, eyesy):
    _surfaces(screen)
    now = time.time()
    gap = now - S["last"]
    S["last"] = now
    dt = getattr(eyesy, "dt", None)                    # stereopsis: whole display refreshes, no jitter
    dt = min(0.1, max(0.0, gap if dt is None else dt))
    S["t"] += dt
    fg = eyesy.color_picker_lfo(eyesy.knob4)
    bg = eyesy.color_picker_bg(eyesy.knob5)
    # pygame's fill() truncates float colours; the kernel gets the same integers, so its background pixels are
    # identical to the engine's fill (which then lets it skip the dark tiles: prefilled)
    bgi = tuple(int(c) for c in bg[:3])
    prefilled = 1.0 if (getattr(eyesy, "auto_clear", False) and S.get("last_bg") == bgi) else 0.0
    S["last_bg"] = bgi
    au = _audio(eyesy)
    level, bass, mid, treble, beat, kick, fft, nfft, fft_hz, wave, nwave, wave_hz, bands, nbands = au
    # the performer's Trigger (button, page, MIDI), not stock's loudness trigger: stereopsis 0.9.1 marks that one
    # eyesy.trig_audio (a hot input fires it nearly every frame); at most every 0.25 s
    trig = bool(eyesy.trig) and not getattr(eyesy, "trig_audio", False) and S["t"] - S["last_trig"] > 0.25
    if trig:
        S["last_trig"] = S["t"]

    lib = S.get("lib")
    if lib is not None and S["native_fmt_ok"]:
        if not S.get("armed"):                         # crash guard: armed by the first native frame (before the
            _arm_guard()                               # set-up below: a crash there is one on trial too)
        if not lib.fx_ready():                         # set up on the engine's thread, the first time it draws
            t_init = time.perf_counter()
            lib.fx_init(S["count"], 7)
            _log("set up in %.0f ms" % ((time.perf_counter() - t_init) * 1000))
        if os.path.exists(os.path.join(S["root"], "KBENCH")) and not S.get("kbench_done"):
            S["kbench_done"] = True
            _kbench(lib)
        if gap > 0.3:                                  # shown again after a pause: no trails from back then
            lib.fx_trails_reset()
        P = S["P"]
        vals = [dt, eyesy.knob1, eyesy.knob2, eyesy.knob3, eyesy.knob4, eyesy.knob5,
                fg[0] / 255.0, fg[1] / 255.0, fg[2] / 255.0, bgi[0] / 255.0, bgi[1] / 255.0, bgi[2] / 255.0,
                level, bass, mid, treble, beat, 1.0 if kick else 0.0, 1.0 if trig else 0.0, prefilled, wave_hz]
        vals += _extras(eyesy, au)
        for i, v in enumerate(vals):
            P[i] = v
        sh = S["shifts"]
        t0 = time.perf_counter()
        screen.lock()
        try:
            rc = lib.fx_frame(screen._pixels_address, S["xr"], S["yr"], screen.get_pitch(), S["down"], sh[0], sh[1],
                              sh[2], S["acc"], S["w"], S["h"], S["bloom"], S["bw"], S["bh"], fft, nfft, fft_hz,
                              wave, nwave, bands, nbands, P)
        finally:
            screen.unlock()
        if rc == 0:
            S["native_frames"] += 1
            if S["native_frames"] == KNOWN_GOOD_FRAMES:
                _clear_guard(S["root"])                                # proven: clear guard + strikes
            if S["debug"]:
                pr = S["prof"]
                el = time.perf_counter() - t0
                pr[0] += el
                pr[1] += 1
                pr[2] = max(pr[2], el)
                pr[3] += el > 0.0145
                if pr[1] == 120:
                    d = (ctypes.c_float * 11)()
                    lib.fx_stats(d, 11)
                    _log("kernel %.2f ms/frame (max %.2f, %d over 14.5), %dx%d (down %d) | bin %.2f bloom %.2f final "
                         "%.2f | tiles lit %.0f bloom %.0f dark %.0f | prims %.0f dropped %.0f | %.3f %.3f %.3f"
                         % (pr[0] / pr[1] * 1000, pr[2] * 1000, pr[3], S["w"], S["h"], S["down"],
                            d[0], d[1], d[2], d[3], d[4], d[5], d[6], d[7], d[8], d[9], d[10]))
                    pr[:] = [0.0, 0, 0.0, 0]
            return
        _log("fx_frame rejected its arguments (rc %d) - using the fallback" % rc)
        S["lib"] = None
    screen.fill(bgi)
    _fallback(screen, eyesy, fg, level, bass)


def _kbench(lib):
    """KBENCH: the renderer's inner loops on this machine, ns per pixel"""
    lib.fx_bench.restype = ctypes.c_float
    lib.fx_bench.argtypes = [ctypes.c_int, ctypes.c_int, ctypes.c_float]
    out = []
    for r in (2.0, 3.0, 4.0, 6.0):
        out.append("sprite r%.0f rgb %.1f rgbx %.1f" % (r, lib.fx_bench(0, 20000, r), lib.fx_bench(1, 20000, r)))
    out.append("tone %.1f" % lib.fx_bench(2, 400, 1.0))
    for r in (1.5, 3.0):
        out.append("line r%.1f %.1f" % (r, lib.fx_bench(3, 20000, r)))
    _log("KBENCH ns/px: " + " | ".join(out))


# ---------------------------------------------------------------------------------------------- fallback
def _fallback(screen, eyesy, fg, level, bass):
    """a globe with a few jagged filaments: while the kernel compiles, or without gcc"""
    xr, yr = screen.get_width(), screen.get_height()
    cx, cy = xr // 2, yr // 2
    r = int(yr * 0.4)
    t = S["t"]
    pygame.draw.circle(screen, tuple(int(c * 0.3) for c in fg[:3]), (cx, cy), r, 1)
    for k in range(3 + int(level * 6)):
        a = t * 0.4 + k * 2.1
        pts = [(cx, cy)]
        for s in range(1, 8):
            u = s / 7.0
            j = math.sin(t * 13 + k * 7 + s * 3) * r * 0.06
            pts.append((int(cx + math.cos(a) * r * u - math.sin(a) * j), int(cy + math.sin(a) * r * u + math.cos(a) * j)))
        pygame.draw.lines(screen, fg, False, pts, 1)
