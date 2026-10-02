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

# 01 - Swarm - Tunnel
#
# free.vet's drone-swarm visualizer on the EYESY, one figure per mode - this one: rings flying away down a tunnel, each shaped by the bass the moment it was born; they roll as
# they go
# Drones pulse with the energy, flash on kicks, twinkle with the treble, and the bass breathes and spins them.
#
#Knob1 - turn: the middle looks straight down the tunnel (the rings fly away from you); turn it either way
#        to swing the tunnel round - side on at a quarter turn, seen from its far end at either end of
#        the knob (the rings come at you)
#Knob2 - lights: a tenth of the drones on the left -> all of them on the right, always evenly spaced
#        (fewer rings, fewer drones a ring); the rings roll on their own
#Knob3 - glow: bloom up to the middle; trails from 60 % up
#Knob4 - colour: turns the palette around the colour wheel
#Knob5 - background color
#Trigger - the swarm bursts outward and reforms (it also bursts on kicks)
# The Trigger here is the performer's: the button, the page, MIDI (stereopsis 0.9.1's eyesy.trig_audio tells them from
# the loudness trigger, which a hot input fires nearly every frame); the bursts with the music come from detected kicks.
#
# The Swarm modes (01 - Swarm - Tunnel .. 06 - Swarm - Cymatics) share one compiled kernel and its state:
# switching between them morphs the drones from one figure into the next.
# Best on stereopsis (its engine analyses the audio: spectrum, waveform, kicks). On the stock engine it still
# runs, reacting to the level (the spectrum figures hold their rest shapes, no kicks).
# The renderer is a C kernel (swarm.c, next to this file) compiled ON the EYESY the first time any of these modes
# runs (~35 s, in the background, one build for all of them; a simple ring shows meanwhile). Files: swarm_*.so =
# compiled cache (safe to delete); .native_trial / .native_strikes = crash guard; NATIVE_OFF (exists = never use
# the kernel; the crash guard writes it if the engine dies twice in a row while the kernel is on trial).
# Dev switches (files containing a number): DRONES (default 4096), DOWN (render scale), SIZE, DEBUG (1 = timings
# to the log), THREADS (2 = the final pass on two cores, the default; 1 = one); ASM (exists = also write the device's
# assembly of swarm.c as swarm.s); BENCH (1 or 2 = synthetic music instead of the input for reproducible timings:
# 1 loud, 2 a mastered club mix - the heaviest load). DOWN: render at 1/DOWN of the surface's size (default: the
# full size on stereopsis, half above 400 lines on the stock engine).
# Written by tools/make_swarm_modes.py from one template, the same for every figure (edit that, not this file).

STYLE = 5
STEREOPSIS = os.environ.get("STEREOPSIS") == "1"     # the engine this runs on (stereopsis sets it)
RIBBON, TERRAIN, ORB, NEBULA, TUNNEL, CYMATICS = (STYLE == 1, STYLE == 2, STYLE == 3, STYLE == 4, STYLE == 5,
                                                  STYLE == 6)
HOLD = RIBBON or ORB or NEBULA                       # the Trigger held breathes the figure
KICK_BURSTS = not (TERRAIN or CYMATICS)              # bursts on detected kicks (the plate pumps on its own)
HOLD_MIN, HOLD_MAX, HOLD_PERIOD = 0.3, 1.9, 3.3      # the breath: x0.3 .. x1.9, 3.3 s a cycle (stock's held-Trigger
                                                     # sine undulates about that fast at 60 fps)
TUNNEL_ROLL = 0.35                                   # the tunnel's rings roll on their own (x 1.2 rad/s)
N_DRONES = 4096
KERNEL_VERSION = 3
N_PARAMS = 33
KNOWN_GOOD_FRAMES = 60
GUARD_SECONDS = 3.0
STYLES = ("corona", "ribbon", "terrain", "orb", "nebula", "tunnel", "cymatics")

S = {}                 # all state lives here (the module is re-imported on reload)


def _log(msg):
    print("[swarm %s] %s" % (STYLES[STYLE], msg))


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


def _find_or_build(root, src_name, prefix):
    """The compiled kernel for this folder's source: from this folder, /tmp, or another mode's folder (modes with the
    same kernel source share one build), else built here. One gcc at a time on the device (a lock in /tmp): gcc at
    -O3 needs ~100 MB and the EYESY has no swap, so every new mode compiling at once at the first boot could exhaust
    its memory. Returns the .so path, or None (why is logged)."""
    src_path = os.path.join(root, src_name)
    src = open(src_path, "rb").read()
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
                if so and os.path.exists(os.path.join(root, "ASM")):   # dev: the device's code, as swarm.s
                    try:
                        subprocess.run(["gcc"] + base + arch + ["-S", "-o", os.path.join(root, "swarm.s"), src_path],
                                       capture_output=True, text=True, timeout=300)
                    except Exception as e:
                        _log("swarm.s failed: %r" % (e,))
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
        so = _find_or_build(root, "swarm.c", "swarm")
        if so is None:
            S["native_state"] = "off: no kernel (see the log)"
            return
        lib = ctypes.CDLL(so)                      # the same file as the other Swarm modes = the same library
        F = ctypes.POINTER(ctypes.c_float)
        lib.sw_init.argtypes = [ctypes.c_int, ctypes.c_int]
        lib.sw_frame.restype = ctypes.c_int
        lib.sw_frame.argtypes = [ctypes.c_void_p, ctypes.c_int, ctypes.c_int, ctypes.c_int, ctypes.c_int,
                                 ctypes.c_int, ctypes.c_int, ctypes.c_int, F, ctypes.c_int, ctypes.c_int,
                                 F, ctypes.c_int, ctypes.c_int, ctypes.c_void_p, ctypes.c_int, ctypes.c_float,
                                 ctypes.c_void_p, ctypes.c_int, F]
        lib.sw_debug.argtypes = [F, ctypes.c_int]
        if lib.sw_version() != KERNEL_VERSION or lib.sw_param_count() != N_PARAMS:
            S["native_state"] = "off: kernel version mismatch"
            return
        lib.sw_threads(int(_num(root, "THREADS", 2)))  # the final pass on two cores (THREADS 1: one, to compare)
        S["n"] = int(max(64, min(lib.sw_max_drones(), _num(root, "DRONES", N_DRONES))))
        S["lib"] = lib                             # sw_init runs on the engine's thread, at the first draw
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
    S["size"] = _num(root, "SIZE", 4.0)
    S["down_file"] = _num(root, "DOWN", 0)
    S["P"] = (ctypes.c_float * N_PARAMS)()
    S["zero"] = (ctypes.c_float * 2)()
    S["wave"] = (ctypes.c_float * 100)()
    S["last"] = time.time()
    S["native_frames"] = 0
    S["native_state"] = "compiling"
    S["lvl"] = 0.0
    S["t"] = 0.0
    S["last_burst"] = 0.0
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
    kick = ph < S["bench_ph"]                          # the phase wrapped: a kick this frame
    S["bench_ph"] = ph
    env = math.exp(-ph * 6.0)
    j = int(t * 30.0) % 16
    if heavy:
        lv, bs, tr = 0.75 + 0.15 * math.sin(t * 2.1), 0.75 + 0.25 * env, 0.5 + 0.2 * math.sin(t * 7.3)
    else:
        lv, bs, tr = 0.55 + 0.2 * math.sin(t * 2.1), 0.4 + 0.55 * env, 0.35 + 0.2 * math.sin(t * 7.3)
    return (lv, bs, 0.45, tr, env, kick, ctypes.addressof(ffts[j]), 1024, 15.625, ctypes.addressof(waves[j]), 1024)


def _audio(eyesy):
    """(level, bass, mid, treble, beat envelope, kick this frame, fft address, bins, bin Hz, wave address,
    samples): stereopsis's analysis when it runs; on the stock engine, a level from the scope samples"""
    if S.get("bench"):
        return _bench_audio()
    if getattr(eyesy, "audio_analysis", False):
        fft, wave = eyesy.audio_fft, eyesy.audio_wave
        return (eyesy.audio_level, eyesy.audio_bass, eyesy.audio_mid, eyesy.audio_treble, eyesy.audio_beat,
                bool(eyesy.beat), ctypes.addressof(fft), len(fft), float(eyesy.audio_fft_hz),
                ctypes.addressof(wave), len(wave))
    ain = eyesy.audio_in
    peak = max(1, max(abs(int(v)) for v in ain)) / 32768.0
    S["lvl"] += (min(1.0, peak * 1.5) - S["lvl"]) * (0.5 if peak > S["lvl"] else 0.1)
    w = S["wave"]
    for i, v in enumerate(ain[:100]):
        w[i] = max(-1.0, min(1.0, v / 32768.0 * 2.0))
    lv = S["lvl"]
    return (lv, lv, lv * 0.6, lv * 0.3, 0.0, False, ctypes.addressof(S["zero"]), 0, 0.0,
            ctypes.addressof(w), len(ain[:100]))


def _bipolar(k, dead=0.04):
    """a knob as -1..1 with a dead zone at its centre (exactly 0 there)"""
    x = (k - 0.5) * 2.0
    return 0.0 if abs(x) < dead else math.copysign((abs(x) - dead) / (1.0 - dead), x)


def _unit(k):
    return min(1.0, max(0.0, float(k)))


# the breath while the Trigger is held: a sine in log scale between HOLD_MIN and HOLD_MAX that starts at the figure's
# own size (1) going up, so a press grows it smoothly first
_LC = 0.5 * (math.log(HOLD_MIN) + math.log(HOLD_MAX))
_LA = 0.5 * (math.log(HOLD_MAX) - math.log(HOLD_MIN))
_PH0 = math.asin(max(-1.0, min(1.0, -_LC / _LA)))


def _breath(ph):
    """the log of the figure's scale at phase ph (0..1) of the breath"""
    return _LC + _LA * math.sin(2.0 * math.pi * ph + _PH0)


def _nebula_push(k):
    """the kicks' push on the nebula by knob 1 (the camera distance): far (0) 2x, the middle 1x, close (1) 0.1x"""
    k = _unit(k)
    return 2.0 ** (1.0 - 2.0 * k) if k <= 0.5 else 10.0 ** (-(2.0 * k - 1.0))


# ---------------------------------------------------------------------------------------------- draw
def draw(screen, eyesy):
    _surfaces(screen)
    now = time.time()
    gap = now - S["last"]
    S["last"] = now
    dt = getattr(eyesy, "dt", None)                    # stereopsis: whole display refreshes, no jitter
    dt = min(0.1, max(0.0, gap if dt is None else dt))
    S["t"] += dt
    k1 = _bipolar(eyesy.knob1)
    k2 = _bipolar(eyesy.knob2, 0.08)
    # the figure's dials (swarm.c v3; these neutral values draw v2's picture)
    pulse = spin = burst_amp = amp = frac = span = 1.0
    tumble = 0.0
    camd, turn, orbit, roll = 2.0 ** (-1.7 * k1), 0.0, k2, 0.0
    cap = 16.0 * min(1.0, camd) ** 0.25                # near drones' size cap eases from 16 px to ~12 close up
    if TUNNEL:                                         # side on, the camera comes in a quarter: the tube fills the width
        turn = k1 * math.pi
        camd, orbit, cap = 1.0 - 0.25 * math.sin(turn) ** 2, 0.0, 16.0
        roll = TUNNEL_ROLL                             # it rolls on its own; knob 2: how many drones are lit
        frac = 0.1 + 0.9 * _unit(eyesy.knob2)
    elif RIBBON:                                       # knob 2: the wave height (0.25x .. 4x, the middle as before)
        orbit = 0.0
        amp = 2.0 ** (4.0 * (_unit(eyesy.knob2) - 0.5))
    elif TERRAIN:                                      # still: no beat swell, no bass spin, no bursts
        pulse = spin = burst_amp = 0.0
    elif ORB:                                          # knob 2: a tumble about the horizontal axis
        orbit, tumble = 0.0, k2
    elif NEBULA:                                       # the kicks' push follows the distance: far big, close small
        pulse = burst_amp = _nebula_push(eyesy.knob1)
    elif CYMATICS:                                     # knob 1: the plate's size (0.4 .. 2.5x the nodes); the view frames it
        camd = 1.0
        span = 2.5 ** (2.0 * _unit(eyesy.knob1) - 1.0)
    # the Trigger held (the button): the figure breathes; let go, it eases back
    held = bool(getattr(eyesy, "key10_status", False))
    hold, smooth = 1.0, 0.0
    if HOLD:
        if held and not S.get("was_held"):
            S["hold_ph"] = 0.0                         # a fresh press: the breath starts at the figure's size
        S["was_held"] = held
        if held:
            S["hold_ph"] = (S.get("hold_ph", 0.0) + dt / HOLD_PERIOD) % 1.0
        env = S.get("hold_env", 0.0)
        env += ((1.0 if held else 0.0) - env) * min(1.0, dt * (6.0 if held else 3.0))
        S["hold_env"] = env
        if env > 1e-3:
            hold = math.exp(env * _breath(S.get("hold_ph", 0.0)))
            smooth = env
    bg = eyesy.color_picker_bg(eyesy.knob5)
    # pygame's fill() truncates float colours; the kernel gets the same integers, so its background pixels are
    # identical to the engine's fill. The engine fills this surface with eyesy.bg_color before draw() when
    # Persist is off - the colour picked here last frame: unchanged, the kernel leaves the dark tiles alone
    bgi = tuple(int(c) for c in bg[:3])
    prefilled = 1.0 if (getattr(eyesy, "auto_clear", False) and S.get("last_bg") == bgi) else 0.0
    S["last_bg"] = bgi
    bg = bgi
    level, bass, mid, treble, beat, kick, fft, nfft, fft_hz, wave, nwave = _audio(eyesy)
    # bursts: with the detected kicks, and the performer's Trigger (not stock's loudness trigger: stereopsis marks it
    # eyesy.trig_audio); none while the Trigger is held here (the breath), at most every 0.45 s
    deliberate = bool(eyesy.trig) and not getattr(eyesy, "trig_audio", False)
    want = (KICK_BURSTS and kick) or deliberate
    if HOLD and (held or S.get("hold_env", 0.0) > 0.05):
        want = False
    trig = False
    if want and burst_amp > 0.0 and S["t"] - S["last_burst"] > 0.45:
        trig = True
        S["last_burst"] = S["t"]

    lib = S.get("lib")
    if lib is not None and S["native_fmt_ok"]:
        if not S.get("armed"):                         # crash guard: armed by the first native frame (before the
            _arm_guard()                               # set-up below: a crash there is one on trial too)
        if lib.sw_ready() != S["n"]:                   # the first Swarm mode to draw sets the shared kernel up
            lib.sw_init(S["n"], 7)
        if gap > 0.3:                                  # shown again after a pause: no trails from back then
            lib.sw_trails_reset()
        P = S["P"]
        vals = (dt, STYLE, orbit, eyesy.knob3, eyesy.knob4, bg[0] / 255.0, bg[1] / 255.0, bg[2] / 255.0,
                level, bass, mid, treble, beat, 1.0 if kick else 0.0, 1.0 if trig else 0.0,
                S["size"], 1.0, 1.5, 1.05, prefilled, camd, turn, roll, cap,
                pulse, spin, burst_amp, hold, smooth, amp, tumble, frac, span)
        for i, v in enumerate(vals):
            P[i] = v
        sh = S["shifts"]
        t0 = time.perf_counter()
        screen.lock()
        try:
            rc = lib.sw_frame(screen._pixels_address, S["xr"], S["yr"], screen.get_pitch(), S["down"], sh[0], sh[1],
                              sh[2], S["acc"], S["w"], S["h"], S["bloom"], S["bw"], S["bh"],
                              fft, nfft, fft_hz, wave, nwave, P)
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
                pr[3] += el > 0.0145                   # frames over ~14.5 ms (the kernel's share of a 60 fps frame)
                if pr[1] == 120:
                    d = (ctypes.c_float * 16)()
                    lib.sw_debug(d, 16)
                    _log("kernel %.2f ms/frame (max %.2f, %d over 14.5), camd x%.2f turn %.2f, %dx%d (down %d) | layout "
                         "%.2f project %.2f splat %.2f bloom %.2f final %.2f | tiles lit %.0f bloom %.0f dark %.0f"
                         % (pr[0] / pr[1] * 1000, pr[2] * 1000, pr[3], camd, turn, S["w"], S["h"], S["down"],
                            d[8], d[9], d[10], d[11], d[12], d[13], d[14], d[15]))
                    pr[:] = [0.0, 0, 0.0, 0]
            return
        _log("sw_frame rejected its arguments (rc %d) - using the fallback" % rc)
        S["lib"] = None
    _fallback(screen, eyesy, bg, level, bass)


# ---------------------------------------------------------------------------------------------- fallback
def _fallback(screen, eyesy, bg, level, bass):
    """a ring of drones breathing with the level: while the kernel compiles, or without gcc"""
    screen.fill(tuple(int(c) for c in bg[:3]))
    xr, yr = screen.get_width(), screen.get_height()
    cx, cy = xr // 2, yr // 2
    n = 120
    base = yr * 0.28 * (1.0 + bass * 0.4)
    t = S["t"]
    col = eyesy.color_picker_lfo(eyesy.knob4)
    rad = max(2, int(yr / 180))
    for i in range(n):
        a = i / n * 2 * math.pi + t * 0.3
        r = base * (1.0 + 0.25 * level * math.sin(i * 0.7 + t * 3.0))
        pygame.draw.circle(screen, col, (int(cx + math.cos(a) * r), int(cy + math.sin(a) * r)), rad)
