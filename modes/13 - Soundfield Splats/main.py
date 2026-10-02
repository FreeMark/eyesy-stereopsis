import os
import math
import time
import random
import ctypes
import hashlib
import platform
import subprocess
import threading
import array
import colorsys
import pygame

# 13 - Soundfield Splats
#
# Sound, slowed ~1000x, moving through a disc of air particles in 3D. Two sources (left and right
# input channels) radiate their signal outward; each particle is pushed along the direction of travel
# by the pressure reaching it - loudness sets the amplitude, a pitch estimate (zero-crossing rate) sets
# the wavelength, compressions glow in the foreground colour and rarefactions in its complement.
# Every particle is a Gaussian splat with depth of field.
#
# The renderer is a small C kernel (splat.c, next to this file) that this mode compiles ON the EYESY
# with gcc the first time it runs (about 3 s, in the background) and calls through ctypes. It writes
# straight into the screen. Until it's ready - or anywhere without gcc, like a desktop emulator - a
# pure-pygame fallback draws fewer particles.
#
#Knob1 - speed of sound (slow, wide waves -> fast, tight waves)
#Knob2 - camera orbit: left of centre spins one way, right the other, centre holds still
#Knob3 - afterglow / trails
#Knob4 - foreground color (compressions; rarefactions get the complementary hue)
#Knob5 - background color
#Trigger - fires a sharp impulse (a clap) from both sources
#
# Persist is ignored: the mode keeps its own trails (Knob3).
# Files in this folder: splat.c (the kernel source), splat_*.so (compiled cache, safe to delete),
# .native_trial/.native_strikes (crash guard), NATIVE_OFF (exists = pure-pygame only; the crash guard
# writes it if the engine dies twice in a row while the kernel is on trial - delete it to retry). Dev switches (files containing
# 1 or 0): DEBUG (timings to the log), BENCH (one-off benchmark), BASELINE (draw nothing, to measure
# the engine), DOWN (render scale, e.g. 2 or 3), PROFILE (split the engine's own per-frame cost into
# fill / blit / flip for PROFILE_FRAMES frames, then undo itself).

N_NATIVE = 3000        # particles for the C renderer (measured budget on the CM3, see OPERATIONS.md)
N_FALLBACK = 380       # particles for the pure-pygame renderer
# accumulate at 1/DOWN of the surface's size (the kernel block-upscales into the screen): its full size on
# stereopsis, which sets the surface's size (scale.json, the page); half on the stock engine, for speed
DOWN_DEFAULT = 1 if os.environ.get("STEREOPSIS") == "1" else 2
HLEN = 320             # history steps (one per frame)
CLOUD_R = 1.5
DISC_HALF_THICK = 0.2
SRC_SEP = 0.42
KNOWN_GOOD_FRAMES = 60 # native frames before the crash guard is cleared
PROFILE_FRAMES = 300   # PROFILE switch: frames to measure before pygame.display.flip is restored
KERNEL_VERSION = 5
PULSE = (1.0, 0.5, -0.3, -0.6, -0.4, -0.2)        # crest then trough, per frame; sums to zero

S = {}                 # all state lives here (the module is re-imported on reload)


def _log(msg):
    print("[soundfield] " + msg)


def _flag(root, name):
    try:
        return open(os.path.join(root, name)).read().strip() == "1"
    except Exception:
        return False


def _num(root, name, default):
    try:
        return float(open(os.path.join(root, name)).read().strip())
    except Exception:
        return default


# ---------------------------------------------------------------------------------------------- native build
def _native_allowed(root):
    """Crash guard. The marker is armed on the first native frame this mode actually DRAWS (not at load:
    every mode loads at boot) and cleared after KNOWN_GOOD_FRAMES. A marker left by a dead engine means
    it died while the kernel was on trial - or someone powered off within ~2 s of selecting the mode.
    Strike 1: pure-pygame for this session only. Strike 2 in a row: NATIVE_OFF (permanent until deleted)."""
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
            return False, "engine died during native trial (strike 1 of 2) - pure-pygame for this session"
    return True, ""


def _compile_and_load(root):
    t0 = time.time()
    try:
        ok, why = _native_allowed(root)
        if not ok:
            _log("native renderer off: " + why)
            S["native_state"] = "off: " + why
            return
        src_path = os.path.join(root, "splat.c")
        if not os.path.exists(src_path):
            S["native_state"] = "off: splat.c missing"
            return
        src = open(src_path, "rb").read()
        mach = platform.machine() or "unknown"
        base = ["-O3", "-fno-math-errno", "-shared", "-fPIC"]
        # -funsafe-math-optimizations lets gcc use NEON for float loops on 32-bit ARM (NEON flushes denormals);
        # unlike -ffast-math it keeps NaN comparisons, which the kernel's guards rely on
        arch = (["-mcpu=cortex-a53", "-mfpu=neon-fp-armv8", "-mfloat-abi=hard", "-funsafe-math-optimizations"]
                if mach.startswith("armv7") else [])
        tag = hashlib.sha1(src + " ".join(base + arch).encode()).hexdigest()[:10]
        out_dirs = [root, "/tmp"]
        so = None
        for d in out_dirs:
            cand = os.path.join(d, "splat_%s_%s.so" % (tag, mach))
            if os.path.exists(cand):
                so = cand
                break
        if so is None:
            for d in out_dirs:
                cand = os.path.join(d, "splat_%s_%s.so" % (tag, mach))
                tmp = cand + ".tmp%d" % os.getpid()
                for flags in (base + arch, base):
                    try:
                        r = subprocess.run(["gcc"] + flags + ["-o", tmp, src_path, "-lm"],
                                           capture_output=True, text=True, timeout=240)
                    except Exception as e:
                        _log("gcc not usable (%s) - fallback renderer" % e)
                        S["native_state"] = "off: no gcc"
                        return
                    if r.returncode == 0:
                        os.replace(tmp, cand)
                        so = cand
                        _log("compiled %s in %.1f s (%s)" % (os.path.basename(cand), time.time() - t0, " ".join(flags)))
                        break
                    _log("gcc failed with %s: %s" % (" ".join(flags), r.stderr.strip()[-300:]))
                if so:
                    break
            if so is None:
                S["native_state"] = "off: compile failed"
                return
            for d in out_dirs:                     # drop stale kernels this mode built earlier
                try:
                    for f in os.listdir(d):
                        if f.startswith("splat_") and f.endswith(".so") and os.path.join(d, f) != so:
                            os.remove(os.path.join(d, f))
                except Exception:
                    pass
        lib = ctypes.CDLL(so)
        F = ctypes.POINTER(ctypes.c_float)
        lib.sf_render.restype = ctypes.c_int
        lib.sf_render.argtypes = [ctypes.c_void_p, ctypes.c_int, ctypes.c_int, ctypes.c_int, ctypes.c_int, ctypes.c_int,
                                  ctypes.c_int, ctypes.c_int, F, ctypes.c_int, ctypes.c_int,
                                  ctypes.c_int, F, F, ctypes.c_int, F, F, F, ctypes.c_int, F]
        if lib.sf_version() != KERNEL_VERSION or lib.sf_param_count() != 28:
            S["native_state"] = "off: kernel version mismatch"
            return
        S["lib"] = lib
        S["native_state"] = "on"
        _log("native renderer ready (%.1f s)" % (time.time() - t0))
    except Exception as e:
        S["native_state"] = "off: %s" % e
        _log("native load failed: %r" % (e,))


# ---------------------------------------------------------------------------------------------- setup
def setup(screen, eyesy):
    S.clear()
    root = eyesy.mode_root if getattr(eyesy, "mode_root", "") else os.path.dirname(os.path.abspath(__file__))
    S["root"] = root
    S["debug"] = _flag(root, "DEBUG")
    S["bench"] = _flag(root, "BENCH")
    S["baseline"] = _flag(root, "BASELINE")
    S["baseline_ms"] = _num(root, "BASELINE_MS", 0.0)
    S["down"] = int(max(1, min(8, _num(root, "DOWN", DOWN_DEFAULT))))
    _profile_unwrap(eyesy)                     # never leave a timing wrapper behind across reloads
    S["profile"] = _flag(root, "PROFILE")
    # The air: a thick disc (spherical wavefronts cut through it as readable 3D ripples) plus a sparse
    # halo above and below for depth. Shuffled so the fallback's first N particles sample both.
    rnd = random.Random(1234)
    cloud = []
    n_disc = int(N_NATIVE * 0.88)
    while len(cloud) < N_NATIVE:
        x, z = rnd.uniform(-1, 1), rnd.uniform(-1, 1)
        if x * x + z * z > 1.0:
            continue
        if len(cloud) < n_disc:
            cloud.append((x * CLOUD_R, rnd.uniform(-1, 1) * DISC_HALF_THICK, z * CLOUD_R, rnd.random()))
        else:
            y = rnd.uniform(-1, 1)
            if x * x + y * y + z * z <= 1.0 and abs(y) * CLOUD_R > DISC_HALF_THICK:
                cloud.append((x * CLOUD_R, y * CLOUD_R * 0.7, z * CLOUD_R, rnd.random()))
    rnd.shuffle(cloud)
    S["pts"] = [c for p in cloud for c in p[:3]]
    S["seeds"] = [p[3] for p in cloud]
    S["pos"] = (ctypes.c_float * len(S["pts"]))(*S["pts"])
    S["seed"] = (ctypes.c_float * len(S["seeds"]))(*S["seeds"])
    S["hist"] = [array.array("f", [0.0] * HLEN), array.array("f", [0.0] * HLEN)]
    S["hist_c"] = (ctypes.c_float * (2 * HLEN))()
    S["src"] = (ctypes.c_float * 6)()
    S["lvl"] = (ctypes.c_float * 2)()
    S["P"] = (ctypes.c_float * 28)()
    S["env"] = [0.0, 0.0]
    S["peak"] = [0.02, 0.02]
    S["slow"] = [0.0, 0.0]
    S["pend"] = [[0.0] * 6, [0.0] * 6]      # scheduled pulse samples per channel
    S["refr"] = [0.0, 0.0]                 # onset refractory timers
    S["phase"] = [0.0, 0.0]
    S["yaw"] = 0.6
    S["t"] = 0.0
    S["last"] = time.time()
    S["dt"] = 1.0 / 30.0
    S["native_frames"] = 0
    S["native_state"] = "compiling"
    S["prof"] = {"py": 0.0, "native": 0.0, "fb": 0.0, "n": 0}
    threading.Thread(target=_compile_and_load, args=(root,), daemon=True).start()


def _surfaces(screen):
    """(Re)create the buffers for this draw surface (size / pixel format)."""
    xr, yr = int(screen.get_width()), int(screen.get_height())
    key = (xr, yr, screen.get_bitsize(), screen.get_masks(), S["down"])
    if S.get("surf_key") == key:
        return
    down = S["down"]
    w, h = (xr + down - 1) // down, (yr + down - 1) // down
    sh = screen.get_shifts()
    S.update(surf_key=key, xr=xr, yr=yr, w=w, h=h, shifts=(sh[0], sh[1], sh[2]))
    S["acc"] = (ctypes.c_float * (w * h * 3))()
    S["native_fmt_ok"] = (screen.get_bytesize() == 4 and hasattr(screen, "_pixels_address")
                          and len({sh[0], sh[1], sh[2]}) == 3 and all(s in (0, 8, 16, 24) for s in sh[:3]))
    if not S["native_fmt_ok"]:
        _log("screen format not usable by the kernel (bytesize %d, shifts %s) - fallback" % (screen.get_bytesize(), sh))
    fw, fh = max(16, xr // 3), max(9, yr // 3)        # fallback renders at 1/3 and nearest-scales up
    S.update(fw=fw, fh=fh, fb=pygame.Surface((fw, fh), 0, screen), fb_tmp=pygame.Surface((fw, fh), 0, screen))
    S["sprites_key"] = None


# ---------------------------------------------------------------------------------------------- audio -> history
def _channel(buf, ch, dt, impulse):
    n = len(buf)
    if n == 0:
        return 0.0
    s2 = 0.0
    zc = 0
    prev = buf[0]
    for v in buf:
        s2 += v * v
        if (v >= 0) != (prev >= 0):
            zc += 1
        prev = v
    raw = math.sqrt(s2 / n) / 32768.0
    # automatic gain: normalise against a peak tracker (instant attack, ~6 s release) over a floor, so a
    # quiet podcast and a hot synth both drive full-strength waves while true silence stays dark
    pk = max(raw, S["peak"][ch] * (1.0 - 0.17 * dt), 0.02)
    S["peak"][ch] = pk
    level = min(1.0, raw / pk)
    env = S["env"][ch]
    env += (level - env) * (0.55 if level > env else 0.10)
    S["env"][ch] = env
    # onset (beat) detector: level jumping well above its slow average -> a sharp shell
    slow = S["slow"][ch]
    S["slow"][ch] = slow + (level - slow) * min(1.0, 2.5 * dt)
    hit = max(0.0, level - slow * 1.4 - 0.12)
    S["refr"][ch] = max(0.0, S["refr"][ch] - dt)
    pend = S["pend"][ch]
    amp = min(1.2, hit * 2.5) + impulse
    if amp > 0.05 and (S["refr"][ch] <= 0.0 or impulse > 0.0):
        # a pressure pulse is bipolar (a clap is a compression then a rarefaction): crest, then trough
        for i, k in enumerate(PULSE):
            pend[i] += amp * k
        S["refr"][ch] = 0.25                  # kicks, not every hi-hat
    zcr = zc / float(max(1, n - 1))
    f_vis = 0.3 + 1.1 * min(1.0, max(0.0, (zcr - 0.02) / 0.38))     # the slowed "pitch", Hz
    ph = S["phase"][ch] + 2.0 * math.pi * f_vis * dt
    S["phase"][ch] = ph % (2.0 * math.pi)
    h = S["hist"][ch]
    h[1:] = h[:-1]
    h[0] = 0.35 * env * math.sin(ph) + pend.pop(0)
    pend.append(0.0)
    return env


def _hue_shift(rgb, amount):
    r, g, b = [min(1.0, max(0.0, c / 255.0)) for c in rgb[:3]]
    hh, ll, ss = colorsys.rgb_to_hls(r, g, b)
    if ss < 0.08:                                   # greys: a darker/lighter grey instead of a hue
        l2 = 0.35 if ll > 0.5 else 0.75
        return (l2, l2, l2)
    return colorsys.hls_to_rgb((hh + amount) % 1.0, ll, ss)


# ---------------------------------------------------------------------------------------------- draw
def draw(screen, eyesy):
    if not S.get("profile"):
        return _draw(screen, eyesy)
    _profile_pre(screen, eyesy)
    w0, c0 = time.perf_counter(), time.thread_time()
    try:
        return _draw(screen, eyesy)
    finally:
        _profile_post(screen, eyesy, time.perf_counter() - w0, time.thread_time() - c0)


def _draw(screen, eyesy):
    t_start = time.perf_counter()
    _surfaces(screen)
    now = time.time()
    dt = min(0.2, max(0.005, now - S["last"]))
    S["last"] = now
    S["dt"] += (dt - S["dt"]) * 0.05
    S["t"] += dt

    # ---- knobs
    k1, k2, k3 = eyesy.knob1, eyesy.knob2, eyesy.knob3
    c_sound = 0.16 + 1.5 * k1                       # world units per second (disc radius 1.5)
    orbit = (k2 - 0.5) * 2.0
    if abs(orbit) < 0.06:
        orbit = 0.0
    S["yaw"] += orbit * 0.7 * dt
    decay = 0.0 if k3 < 0.02 else 0.35 + 0.6 * k3 ** 1.5
    fg = eyesy.color_picker_lfo(eyesy.knob4)
    bg = eyesy.color_picker_bg(eyesy.knob5)
    cpos = tuple(min(1.0, max(0.0, c / 255.0)) for c in fg[:3])
    cneg = _hue_shift(fg, 0.47)
    cbase = tuple(0.18 * (a + b) * 0.5 + 0.04 for a, b in zip(cpos, cneg))

    # ---- audio into the slowed-down history (trigger = a clap from both sources)
    imp = 1.4 if eyesy.trig else 0.0
    left = eyesy.audio_in
    right = getattr(eyesy, "audio_in_r", None) or left
    lvl_l = _channel(left, 0, dt, imp)
    lvl_r = _channel(right, 1, dt, imp)

    # ---- sources drift gently
    t = S["t"]
    wob = 0.12
    src = S["src"]
    src[0], src[1], src[2] = -SRC_SEP + wob * math.sin(t * 0.21), wob * math.sin(t * 0.17 + 1.0), wob * math.cos(t * 0.13)
    src[3], src[4], src[5] = SRC_SEP + wob * math.sin(t * 0.19 + 2.0), wob * math.cos(t * 0.23), wob * math.sin(t * 0.11 + 0.5)
    S["lvl"][0] = min(1.0, lvl_l * 1.4 + imp * 0.5)
    S["lvl"][1] = min(1.0, lvl_r * 1.4 + imp * 0.5)

    h = S["h"]
    camd = 3.6
    params = [S["yaw"], 0.62 + 0.10 * math.sin(t * 0.07), camd, 1.75 * h, camd, 1.6 * (h / 360.0),
              0.16, 0.03, 2.2, 0.95 * (h / 360.0), decay, 1.3,
              (1.0 / S["dt"]) / c_sound, 0.9, 0.45, 2.6,
              cpos[0], cpos[1], cpos[2], cneg[0], cneg[1], cneg[2], cbase[0], cbase[1], cbase[2],
              bg[0] / 255.0, bg[1] / 255.0, bg[2] / 255.0]
    t_py = time.perf_counter()

    if S["baseline"]:                                # measure the engine: draw (almost) nothing
        screen.fill(tuple(int(c) for c in bg[:3]))
        if S["baseline_ms"] > 0:
            end = time.perf_counter() + S["baseline_ms"] / 1000.0
            while time.perf_counter() < end:
                pass
        _prof(t_start, t_py, None, 0.0, eyesy)
        return

    lib = S.get("lib")
    if lib is not None and S["native_fmt_ok"]:
        P = S["P"]
        for i, v in enumerate(params):
            P[i] = v
        hc = S["hist_c"]
        hc[:HLEN] = S["hist"][0]
        hc[HLEN:] = S["hist"][1]
        sh = S["shifts"]
        if not S.get("armed"):                   # crash guard: armed by the first native frame
            S["armed"] = True
            try:
                with open(os.path.join(S["root"], ".native_trial"), "w") as f:
                    f.write(str(os.getpid()))
            except Exception:
                pass
        screen.lock()
        try:
            rc = lib.sf_render(screen._pixels_address, S["xr"], S["yr"], screen.get_pitch(), S["down"], sh[0], sh[1], sh[2],
                               S["acc"], S["w"], S["h"], N_NATIVE, S["pos"], S["seed"], 2, src, S["lvl"],
                               hc, HLEN, P)
        finally:
            screen.unlock()
        t_nat = time.perf_counter()
        if rc == 0:
            S["native_frames"] += 1
            if S["native_frames"] == KNOWN_GOOD_FRAMES:
                for f in (".native_trial", ".native_strikes"):            # proven: clear guard + strikes
                    try:
                        os.remove(os.path.join(S["root"], f))
                    except Exception:
                        pass
            _prof(t_start, t_py, t_nat, None, eyesy)
            if S["bench"] and not S.get("bench_done"):
                S["bench_done"] = True
                _bench(screen, eyesy)
            return
        _log("sf_render rejected its arguments (rc %d) - using the fallback" % rc)
        S["lib"] = None
    t0 = time.perf_counter()
    _fallback(screen, params, decay, cpos, cneg, cbase, bg)
    _prof(t_start, t_py, None, time.perf_counter() - t0, eyesy)


# ---------------------------------------------------------------------------------------------- fallback (pure pygame)
def _make_sprites(cpos, cneg, cbase):
    key = tuple(int(c * 15) for c in tuple(cpos) + tuple(cneg) + tuple(cbase))
    if S.get("sprites_key") == key:
        return
    S["sprites_key"] = key
    sprites = {}
    for ci, col in enumerate((cpos, cneg, cbase)):
        for r in range(1, 9):
            for lvl in range(4):
                gain = (0.25, 0.45, 0.7, 1.0)[lvl]
                size = 6 * r + 1
                sp = pygame.Surface((size, size))
                sp.fill((0, 0, 0))
                for rr in range(3 * r, 0, -1):
                    w = math.exp(-0.5 * (rr / float(r)) ** 2) * gain
                    pygame.draw.circle(sp, tuple(min(255, int(255 * c * w)) for c in col), (size // 2, size // 2), rr)
                sprites[(ci, r, lvl)] = sp
    S["sprites"] = sprites


def _fallback(screen, params, decay, cpos, cneg, cbase, bg):
    fb = S["fb"]
    if decay > 0:
        d = int(255 * decay)
        fb.fill((d, d, d), special_flags=pygame.BLEND_MULT)
    else:
        fb.fill((0, 0, 0))
    _make_sprites(cpos, cneg, cbase)
    sprites = S["sprites"]
    yaw, pitch, camd = params[0], params[1], params[2]
    fov = 1.75 * S["fh"]
    spu, r0, dispg, sat = params[12], params[13], params[6], params[14]
    cyaw, syaw, cp, sp = math.cos(yaw), math.sin(yaw), math.cos(pitch), math.sin(pitch)
    hw, hh = S["fw"] * 0.5, S["fh"] * 0.5
    src = S["src"]
    hists = S["hist"]
    pts = S["pts"]
    blits = []
    add = pygame.BLEND_ADD
    for i in range(N_FALLBACK):
        x, y, z = pts[3 * i], pts[3 * i + 1], pts[3 * i + 2]
        pres = dx = dy = dz = 0.0
        for s in (0, 1):
            ex, ey, ez = x - src[3 * s], y - src[3 * s + 1], z - src[3 * s + 2]
            r = math.sqrt(ex * ex + ey * ey + ez * ez) + 1e-6
            k = r * spu
            ki = int(k)
            if ki >= HLEN - 1:
                continue
            f = k - ki
            hs = hists[s]
            v = (hs[ki] * (1 - f) + hs[ki + 1] * f) * (r0 / (r + r0))
            pres += v
            q = v / r
            dx += ex * q
            dy += ey * q
            dz += ez * q
        x += dx * dispg
        y += dy * dispg
        z += dz * dispg
        x1 = cyaw * x + syaw * z
        z1 = -syaw * x + cyaw * z
        y2 = cp * y - sp * z1
        zc = sp * y + cp * z1 + camd
        if zc < 0.05:
            continue
        sx = hw + fov * x1 / zc
        sy = hh - fov * y2 / zc
        tt = min(1.0, abs(pres) / sat)
        ci = 2 if tt < 0.15 else (0 if pres >= 0 else 1)
        lvl = min(3, int(tt * 4))
        r = max(1, min(8, int(1.6 * camd / zc + 0.5)))
        spr = sprites[(ci, r, lvl)]
        o = spr.get_width() // 2
        blits.append((spr, (int(sx) - o, int(sy) - o), None, add))
    fb.blits(blits, doreturn=False)
    tmp = S["fb_tmp"]
    tmp.blit(fb, (0, 0))
    tmp.fill(tuple(int(c) for c in bg[:3]), special_flags=pygame.BLEND_ADD)
    pygame.transform.scale(tmp, (S["xr"], S["yr"]), screen)   # nearest: smoothscale costs ~38 ms on the CM3


# ---------------------------------------------------------------------------------------------- engine profile
# PROFILE switch: where does the ENGINE's per-frame time go? Each frame the engine polls OSC, MIDI and
# events, updates knobs, fills the mode surface, calls draw(), blits the surface onto the display surface,
# draws the OSD if it is on, calls pygame.display.flip(), clears flags and waits in clock.tick(30).
# We time the same fill and blit on the real surfaces (the kernel overwrites every pixel afterwards, so the
# picture is unchanged) and wrap every other step for wall AND thread-CPU time: CPU ~= wall means the CPU is
# doing the work, CPU << wall means it is waiting. Every wrapper removes itself after PROFILE_FRAMES frames,
# after 120 s, on any error, and on every setup(), so none can outlive the measurement (reload-safe: the
# wrappers carry _sf_prof/_sf_orig, so a re-imported copy of this module can still find and remove them).
_PROFILE_FUNCS = (("pygame.event", "get"), ("osc", "recv"), ("midi", "recv_ttymidi"), ("midi", "recv_usbmidi"),
                  ("osd", "render_overlay_480"), ("pygame.display", "flip"))
_PROFILE_METHODS = ("update_knobs_and_notes", "update_key_repeater", "check_gain_knob", "knob_seq_run",
                    "set_knobs", "update_scene_save_key", "clear_flags")


def _med(xs):
    s = sorted(xs)
    return s[len(s) // 2] if s else 0.0


def _profile_targets(eyesy):
    import sys
    out = []
    for modname, attr in _PROFILE_FUNCS:
        m = sys.modules.get(modname)
        if m is not None and hasattr(m, attr):
            out.append((m, attr, False, modname.split(".")[-1] + "." + attr))
    if eyesy is not None:
        for attr in _PROFILE_METHODS:
            if hasattr(eyesy, attr):
                out.append((eyesy, attr, True, attr))
    return out


def _restore(owner, attr, orig, inst):
    try:
        if inst:
            owner.__dict__.pop(attr, None)          # the class method shows through again
        elif getattr(getattr(owner, attr, None), "_sf_prof", False):
            setattr(owner, attr, orig)
    except Exception:
        pass


def _profile_unwrap(eyesy=None):
    import sys
    if eyesy is None:
        eyesy = (S.get("pf") or {}).get("eyesy")
    for owner, attr, inst, _ in _profile_targets(eyesy):
        cur = owner.__dict__.get(attr) if inst else getattr(owner, attr, None)
        if getattr(cur, "_sf_prof", False):
            _restore(owner, attr, cur._sf_orig, inst)
    main = sys.modules.get("__main__")
    clk = getattr(main, "clocker", None)
    if getattr(clk, "_sf_prof", False):
        main.clocker = clk._sf_orig


def _timed(label, fn, owner, attr, inst):
    def w(*a, **k):
        w0, c0 = time.perf_counter(), time.thread_time()
        try:
            return fn(*a, **k)
        finally:
            try:
                pf = S.get("pf")
                if pf is None or pf.get("done") or w0 - pf["t0"] > 120.0:
                    _restore(owner, attr, fn, inst)
                else:
                    lst = pf["stage"].setdefault(label, [])
                    if len(lst) < 4096:
                        lst.append((time.perf_counter() - w0, time.thread_time() - c0))
            except Exception:
                pass
    w._sf_prof = True
    w._sf_orig = fn
    return w


class _ClockProxy(object):
    """Stands in for the engine's pygame Clock so clock.tick(30) can be timed."""
    _sf_prof = True

    def __init__(self, clk):
        self._sf_orig = clk

    def tick(self, *a):
        w0, c0 = time.perf_counter(), time.thread_time()
        try:
            return self._sf_orig.tick(*a)
        finally:
            try:
                import sys
                pf = S.get("pf")
                if pf is None or pf.get("done") or w0 - pf["t0"] > 120.0:
                    sys.modules["__main__"].clocker = self._sf_orig
                else:
                    lst = pf["stage"].setdefault("clock.tick", [])
                    if len(lst) < 4096:
                        lst.append((time.perf_counter() - w0, time.thread_time() - c0))
            except Exception:
                pass

    def __getattr__(self, name):
        return getattr(self._sf_orig, name)


def _profile_wrap(eyesy):
    import sys
    for owner, attr, inst, label in _profile_targets(eyesy):
        cur = getattr(owner, attr)
        if getattr(cur, "_sf_prof", False):
            continue
        setattr(owner, attr, _timed(label, cur, owner, attr, inst))
    main = sys.modules.get("__main__")
    clk = getattr(main, "clocker", None)
    if clk is not None and hasattr(clk, "tick") and not getattr(clk, "_sf_prof", False):
        main.clocker = _ClockProxy(clk)


def _profile_info(screen):
    try:
        disp = pygame.display.get_surface()
        info = pygame.display.Info()
        _log("PROFILE display driver %s, SDL %s, pygame %s, window %s"
             % (pygame.display.get_driver(), ".".join(str(v) for v in pygame.get_sdl_version()), pygame.version.ver,
                pygame.display.get_window_size()))
        for name, s in (("display", disp), ("mode_screen", screen)):
            if s is not None:
                _log("PROFILE %s surface %dx%d, %d bpp, pitch %d, masks %s, flags 0x%x"
                     % (name, s.get_width(), s.get_height(), s.get_bitsize(), s.get_pitch(), s.get_masks(), s.get_flags()))
        _log("PROFILE display is mode_screen: %s | Info hw=%s wm=%s current=%dx%d"
             % (disp is screen, info.hw, info.wm, info.current_w, info.current_h))
        _log("PROFILE env %s" % ({k: v for k, v in os.environ.items() if k.startswith(("SDL", "PYGAME"))},))
        n = screen.get_pitch() * screen.get_height()
        a, b = ctypes.create_string_buffer(n), ctypes.create_string_buffer(n)
        ctypes.memset(a, 1, n)
        ctypes.memset(b, 2, n)
        t0 = time.perf_counter()
        for _ in range(5):
            ctypes.memmove(b, a, n)
        t1 = time.perf_counter()
        for _ in range(5):
            ctypes.memset(a, 3, n)
        t2 = time.perf_counter()
        _log("PROFILE libc on %.2f MB: memmove %.2f ms (%.0f MB/s), memset %.2f ms (%.0f MB/s)"
             % (n / 1e6, (t1 - t0) * 200, n * 5 / (t1 - t0) / 1e6, (t2 - t1) * 200, n * 5 / (t2 - t1) / 1e6))
    except Exception as e:
        _log("PROFILE info failed: %r" % (e,))


def _profile_pre(screen, eyesy):
    now = time.perf_counter()
    pf = S.get("pf")
    if pf is None:
        pf = S["pf"] = {"t0": now, "n": 0, "last": None, "rows": [], "stage": {}, "cur": {}, "eyesy": eyesy}
        _profile_info(screen)
        _profile_wrap(eyesy)
        _log("PROFILE started (%d frames), OSD %s" % (PROFILE_FRAMES, "ON" if getattr(eyesy, "show_osd", False) else "off"))
    if pf.get("done"):
        return
    interval = (now - pf["last"]) if pf["last"] is not None else None
    pf["last"] = now
    w0, c0 = time.perf_counter(), time.thread_time()
    screen.fill((0, 0, 0))                               # the engine's auto-clear fill, timed
    pf["cur"] = {"interval": interval, "fill_w": time.perf_counter() - w0, "fill_c": time.thread_time() - c0}


def _profile_report(pf):
    rows = pf["rows"][-60:]
    fw, fc = _med([r["fill_w"] for r in rows]), _med([r["fill_c"] for r in rows])
    bw, bc = _med([r["blit_w"] for r in rows]), _med([r["blit_c"] for r in rows])
    dw, dc = _med([r["draw_w"] for r in rows]), _med([r["draw_c"] for r in rows])
    ups = [r["upd_w"] for r in rows if r.get("upd_w") is not None]
    frame = _med([r["interval"] for r in rows if r["interval"] is not None])
    parts, st_w = [], 0.0
    for label in sorted(pf["stage"]):
        lst = pf["stage"][label][-60:]
        sw, sc = _med([x[0] for x in lst]), _med([x[1] for x in lst])
        st_w += sw
        parts.append("%s %.2f/%.2f" % (label, 1000 * sw, 1000 * sc))
    rest = frame - dw - 2 * (fw + bw) - st_w             # our timed fill+blit duplicates are inside the interval
    _log("PROFILE ms (median of 60, wall/cpu): frame %.1f | mode draw %.1f/%.1f | fill %.2f/%.2f | blit %.2f/%.2f"
         " | update(64x64) %s | unaccounted %.2f" % (1000 * frame, 1000 * dw, 1000 * dc, 1000 * fw, 1000 * fc,
                                                     1000 * bw, 1000 * bc, ("%.2f" % (1000 * _med(ups))) if ups else "-",
                                                     1000 * rest))
    _log("PROFILE stages: " + " | ".join(parts))


def _profile_post(screen, eyesy, draw_w, draw_c):
    pf = S.get("pf")
    if pf is None or pf.get("done"):
        return
    try:
        disp = pygame.display.get_surface()
        bw = bc = 0.0
        if disp is not None and disp is not screen:
            w0, c0 = time.perf_counter(), time.thread_time()
            disp.blit(screen, (0, 0))                    # the engine's blit to the display surface, timed
            bw, bc = time.perf_counter() - w0, time.thread_time() - c0
        uw = None
        if pf["n"] % 15 == 7:                            # does a tiny update() cost less than a full flip?
            w0 = time.perf_counter()
            pygame.display.update(pygame.Rect(0, 0, 64, 64))
            uw = time.perf_counter() - w0
        cur = pf["cur"]
        cur.update(blit_w=bw, blit_c=bc, draw_w=draw_w, draw_c=draw_c, upd_w=uw)
        pf["rows"].append(cur)
        pf["n"] += 1
        if pf["n"] % 60 == 0:
            _profile_report(pf)
        if pf["n"] >= PROFILE_FRAMES:
            pf["done"] = True
            _profile_unwrap(eyesy)
            _log("PROFILE done after %d frames; every wrapper removed" % pf["n"])
    except Exception as e:
        pf["done"] = True
        _profile_unwrap(eyesy)
        _log("PROFILE aborted: %r" % (e,))


# ---------------------------------------------------------------------------------------------- instrumentation
def _prof(t_start, t_py, t_nat, t_fb, eyesy):
    if not S["debug"]:
        return
    p = S["prof"]
    p["py"] += t_py - t_start
    if t_nat is not None:
        p["native"] += t_nat - t_py
    if t_fb is not None:
        p["fb"] += t_fb
    p["n"] += 1
    if p["n"] >= 150:
        n = p["n"]
        _log("fps %.1f | per frame: python %.2f ms, native %.2f ms, fallback %.2f ms | %s, down %d, %d particles"
             % (getattr(eyesy, "fps", 0.0), 1000 * p["py"] / n, 1000 * p["native"] / n, 1000 * p["fb"] / n,
                S["native_state"], S["down"], N_NATIVE))
        for k in p:
            p[k] = 0.0 if k != "n" else 0


def _bench(screen, eyesy):
    """One-off timing of the kernel at two scales and several particle counts, into a scratch surface."""
    lib = S["lib"]
    P = S["P"]
    scratch = pygame.Surface((S["xr"], S["yr"]), 0, screen)
    sh = S["shifts"]
    res = []
    for down in (2, 3):
        w, h = (S["xr"] + down - 1) // down, (S["yr"] + down - 1) // down
        acc = (ctypes.c_float * (w * h * 3))()
        P[3] = 1.75 * h
        P[5] = 1.6 * (h / 360.0)
        P[9] = 0.95 * (h / 360.0)
        for n in (0, 1500, 3000):
            scratch.lock()
            t0 = time.perf_counter()
            for _ in range(5):
                lib.sf_render(scratch._pixels_address, S["xr"], S["yr"], scratch.get_pitch(), down, sh[0], sh[1], sh[2],
                              acc, w, h, n, S["pos"], S["seed"], 2, S["src"], S["lvl"], S["hist_c"], HLEN, P)
            scratch.unlock()
            res.append("down %d (%dx%d) n=%d: %.2f ms" % (down, w, h, n, (time.perf_counter() - t0) * 200))
    for line in res:
        _log("BENCH " + line)
