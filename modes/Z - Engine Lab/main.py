import os
import re
import json
import sys
import time
import ctypes
import hashlib
import platform
import subprocess
import threading
import pygame

# Z - Engine Lab
#
# A development mode for the EYESY engine fork stereopsis (its Phase 1). It can
#   METER - log the engine's real frame rate and present time for whichever mode is on screen
#   PROBE - log what the display hardware reports (read-only)
#   TURBO - replace the engine's display path: instead of SDL uploading every frame as a GL texture
#           (~17 ms at 720p), copy it into a KMS scanout buffer and page-flip it (turbo.c, compiled
#           here with gcc the first time, like Soundfield Splats' kernel)
#   SCALE - with TURBO on: "WxH" (e.g. 640x360) makes EVERY mode render at that size, and the display
#           hardware scales it to full screen on an overlay plane (no CPU cost). Each mode is set up
#           again for the size it draws at. "0" = off (full size). The OSD is not visible while on.
#   STEREOPSIS - hand this engine process over to stereopsis, the faster engine in
#           /sdcard/stereopsis/engine: "1" = when this mode is DRAWN,
#           "boot" = early in every boot, "0" = never. A build that dies before 300 good frames leaves
#           /sdcard/stereopsis/.trial behind and the switch goes back to 0, so the next start is the
#           stock engine again. Under stereopsis this mode only shows its status.
#           Switching engines (Shift + Persist held for a second, or the page) sets it too: boot when
#           switching to stereopsis, 0 when switching to the stock engine - the EYESY keeps starting with
#           the engine chosen last (see "switching engines" below).
# Switches are files in this folder holding 1 or 0; FPSCAP holds the frame cap while TURBO is on
# (default 60; the stock engine caps at 30). Nothing happens at boot: switches act only while this
# mode is DRAWN, and METER/TURBO then stay on for every other mode until the engine restarts.
# An engine restart always brings back the stock display path.
#
#Knob1 - not used
#Knob2 - not used
#Knob3 - not used
#Knob4 - not used
#Knob5 - not used

TURBO_VERSION = 2
FORK_DIR = "/sdcard/stereopsis"
FORK_MAIN = FORK_DIR + "/engine/main.py"
UNDER_FORK = os.environ.get("STEREOPSIS") == "1"     # set by stereopsis's main.py
TRIAL_FRAMES = 120        # turbo frames before the crash marker is cleared
METER_EVERY = 2.0         # seconds between METER log lines
S = {}                    # this module instance's state (the module is re-imported on reload)


def _log(msg):
    print("[lab] " + msg)


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


def _write(root, name, text):
    try:
        with open(os.path.join(root, name), "w") as f:
            f.write(text)
    except Exception as e:
        _log("could not write %s: %r" % (name, e))


def _lab():
    """State that must survive this module being re-imported lives on the pygame module."""
    L = getattr(pygame, "_eyesy_lab", None)
    if L is None:
        L = {"orig_flip": None, "orig_clock": None, "lib": None, "fd": -1, "turbo": False, "retired": False,
             "meter": False, "failed": "", "eyesy": None, "root": None, "fpscap": 60, "turbo_frames": 0,
             "m": {"mode": None}, "last_meter": "", "native": "not built", "prev_stats": None}
        pygame._eyesy_lab = L
    return L


# ---------------------------------------------------------------------------------------------- native build
def _compile_and_load(root, L):
    t0 = time.time()
    try:
        src_path = os.path.join(root, "turbo.c")
        if not os.path.exists(src_path):
            L["native"] = "off: turbo.c missing"
            return
        src = open(src_path, "rb").read()
        mach = platform.machine() or "unknown"
        flags = ["-O2", "-D_FILE_OFFSET_BITS=64", "-Wall", "-shared", "-fPIC", "-pthread"]
        tag = hashlib.sha1(src + " ".join(flags).encode()).hexdigest()[:10]
        so = None
        for d in (root, "/tmp"):
            cand = os.path.join(d, "turbo_%s_%s.so" % (tag, mach))
            if os.path.exists(cand):
                so = cand
                break
        if so is None:
            for d in (root, "/tmp"):
                cand = os.path.join(d, "turbo_%s_%s.so" % (tag, mach))
                tmp = cand + ".tmp%d" % os.getpid()
                try:
                    r = subprocess.run(["gcc"] + flags + ["-o", tmp, src_path], capture_output=True, text=True, timeout=240)
                except Exception as e:
                    L["native"] = "off: no gcc (%s)" % e
                    return
                if r.returncode == 0:
                    os.replace(tmp, cand)
                    so = cand
                    _log("compiled %s in %.1f s" % (os.path.basename(cand), time.time() - t0))
                    if r.stderr.strip():
                        _log("gcc warnings: " + r.stderr.strip()[-400:])
                    break
                _log("gcc failed in %s: %s" % (d, r.stderr.strip()[-600:]))
            if so is None:
                L["native"] = "off: compile failed"
                return
            for d in (root, "/tmp"):                 # drop stale builds of older turbo.c versions
                try:
                    for f in os.listdir(d):
                        if f.startswith("turbo_") and f.endswith(".so") and os.path.join(d, f) != so:
                            os.remove(os.path.join(d, f))
                except Exception:
                    pass
        lib = ctypes.CDLL(so)
        lib.tb_version.restype = ctypes.c_int
        lib.tb_is_master.argtypes = [ctypes.c_int]
        lib.tb_probe.argtypes = [ctypes.c_int, ctypes.c_char_p, ctypes.c_int]
        lib.tb_init.argtypes = [ctypes.c_int, ctypes.c_int, ctypes.c_int, ctypes.c_int]
        lib.tb_present.argtypes = [ctypes.c_void_p, ctypes.c_int, ctypes.c_int, ctypes.c_int]
        lib.tb_stats.argtypes = [ctypes.POINTER(ctypes.c_double), ctypes.c_int]
        lib.tb_plane_init.argtypes = [ctypes.c_int] * 6
        lib.tb_plane_present.argtypes = [ctypes.c_void_p, ctypes.c_int, ctypes.c_int, ctypes.c_int]
        lib.tb_plane_stats.argtypes = [ctypes.POINTER(ctypes.c_double), ctypes.c_int]
        if lib.tb_version() != TURBO_VERSION:
            L["native"] = "off: turbo.c version mismatch"
            return
        L["lib"] = lib
        L["native"] = "ready"
        _log("turbo.c ready (%.1f s)" % (time.time() - t0))
    except Exception as e:
        L["native"] = "off: %r" % (e,)
        _log("turbo.c load failed: %r" % (e,))


# ---------------------------------------------------------------------------------------------- crash guard
def _trial_check(root):
    """TURBO arms .turbo_trial when it takes over; TRIAL_FRAMES good frames clear it. A marker left by a
    dead engine means the engine died while TURBO was on trial: switch TURBO off so it is not retried."""
    marker = os.path.join(root, ".turbo_trial")
    if not os.path.exists(marker):
        return
    try:
        pid = int(open(marker).read().strip() or "0")
    except Exception:
        pid = 0
    if pid != os.getpid() and not os.path.exists("/proc/%d" % pid):
        _write(root, "TURBO", "0\n")
        _log("the engine died while TURBO was on trial - TURBO switched off (file TURBO = 0)")
        try:
            os.remove(marker)
        except Exception:
            pass


def _trial_arm(L):
    _write(L["root"], ".turbo_trial", str(os.getpid()))


def _trial_clear(L):
    try:
        os.remove(os.path.join(L["root"], ".turbo_trial"))
    except Exception:
        pass


# ---------------------------------------------------------------------------------------------- fork launch
def _fork_mode(root):
    try:
        return open(os.path.join(root, "STEREOPSIS")).read().strip()
    except Exception:
        return "0"


def _fork_guard(root):
    """False (and STEREOPSIS -> 0) when stereopsis died on trial: its marker names a process that is gone."""
    marker = os.path.join(FORK_DIR, ".trial")
    if not os.path.exists(marker):
        return True
    try:
        pid = int(open(marker).read().strip() or "0")
    except Exception:
        pid = 0
    if pid != os.getpid() and not os.path.exists("/proc/%d" % pid):
        _write(root, "STEREOPSIS", "0\n")
        try:
            os.remove(marker)
        except Exception:
            pass
        _log("STEREOPSIS: it died before 300 good frames last time - switched off (file STEREOPSIS = 0)")
        return False
    return True


def _exec_fork(root, why):
    """Replace this (stock) engine process with stereopsis: same PID, so systemd does not notice."""
    if UNDER_FORK or not os.path.exists(FORK_MAIN):
        _log("STEREOPSIS: not launching (%s)" % ("already running it" if UNDER_FORK else FORK_MAIN + " missing"))
        return
    _write(FORK_DIR, ".trial", str(os.getpid()))
    main = sys.modules.get("__main__")
    ap = getattr(main, "audio_process", None)        # it holds the audio input: stereopsis starts its own
    try:
        if ap is not None and ap.is_alive():
            ap.terminate()
            ap.join(2)
    except Exception as e:
        _log("STEREOPSIS: stopping the audio process: %r" % (e,))
    _log("STEREOPSIS: handing this process to stereopsis (%s)" % why)
    try:
        sys.stdout.flush()
        sys.stderr.flush()
    except Exception:
        pass
    try:
        maxfd = min(int(os.sysconf("SC_OPEN_MAX")), 65536)
    except Exception:
        maxfd = 4096
    try:
        os.closerange(3, maxfd)                    # SDL's DRM fds, the OSC socket, MIDI ports: stereopsis opens its own
        os.chdir(os.path.dirname(FORK_MAIN))
        os.execv(sys.executable, [sys.executable, "-u", FORK_MAIN])
    except Exception:
        os._exit(3)                                 # descriptors are gone: exit, systemd restarts the stock engine


def _boot_hook():
    """STEREOPSIS = boot: launch it while the stock engine is still importing modes (before ~20 s of setups).
    Only during boot: the stock main module defines `start` once its init is done, and a later re-import
    (the editor's Reload) must not launch anything. Staying on the stock engine: switching engines is set up."""
    if UNDER_FORK:
        return
    main = sys.modules.get("__main__")
    if main is None or hasattr(main, "start"):
        return
    root = os.path.dirname(os.path.abspath(__file__))
    if _fork_mode(root) == "boot" and _fork_guard(root):
        _exec_fork(root, "STEREOPSIS=boot")
    _swap_install(root)


# ---------------------------------------------------------------------------------------------- switching engines
# The stock engine with stereopsis installed: Shift + Persist held for a second, or the button on the small page this
# serves where stereopsis's page is (http://<the EYESY>:8081/), hands this process to stereopsis on the mode on
# screen. Where the EYESY is goes into /sdcard/stereopsis/RETURN (mode, Persist, palettes, OSD, gain), and
# STEREOPSIS here goes to boot: the EYESY keeps starting with the engine chosen last. stereopsis switches back the
# same way (its main.py, swap_to_stock): it exits, systemd starts the stock engine, and the first frame here opens
# what it left in RETURN. The stock engine is otherwise untouched: two methods of its eyesy object are wrapped, the
# key handler (to see the hold) and update_key_repeater (to act once a frame, on the engine's own thread).
# /sdcard/stereopsis/SWAP = 0 turns switching off; PREVIEW = 0 / CONTROLS = 0 / PREVIEW_PORT apply to the page.
SWAP_HOLD_S = 1.0


def _fork_flag(name, default=None):
    try:
        return open(os.path.join(FORK_DIR, name)).read().strip()
    except Exception:
        return default


def _swap_enabled():
    return os.path.exists(FORK_MAIN) and _fork_flag("SWAP", "1") != "0"


def _write_atomic(path, text):
    tmp = path + ".tmp"
    with open(tmp, "w") as f:
        f.write(text)
        f.flush()
        os.fsync(f.fileno())
    os.replace(tmp, path)


def _read_return(ey):
    """the settings stereopsis left in RETURN, checked (what does not fit is dropped); the file is removed"""
    path = os.path.join(FORK_DIR, "RETURN")
    try:
        with open(path) as f:
            r = json.load(f)
    except FileNotFoundError:
        return None
    except Exception as e:
        _log("RETURN unreadable (%r): ignored" % (e,))
        r = None
    try:
        os.remove(path)
    except Exception:
        pass
    if not isinstance(r, dict):
        return None
    out = {}
    if isinstance(r.get("mode"), str) and r["mode"] in ey.mode_names:
        out["mode"] = r["mode"]
    for k in ("persist", "osd"):
        if isinstance(r.get(k), bool):
            out[k] = r[k]
    for k in ("fg", "bg"):
        if isinstance(r.get(k), int) and not isinstance(r.get(k), bool) and 0 <= r[k] < len(ey.palettes):
            out[k] = r[k]
    g = r.get("gain")
    if isinstance(g, (int, float)) and not isinstance(g, bool) and g == g and 0.0 <= g <= 1.0:
        out["gain"] = float(g)
    return out


def _apply_return(ey, r):
    if "mode" in r:
        ey.set_mode_by_name(r["mode"])
    if "persist" in r:
        ey.auto_clear = not r["persist"]
    if "fg" in r:
        ey.fg_palette = r["fg"]
    if "bg" in r:
        ey.bg_palette = r["bg"]
    if "osd" in r:
        ey.set_osd(r["osd"])
    if "gain" in r:
        ey.config["audio_gain"] = r["gain"]          # as it was, not saved: config.json keeps what was saved
        ey.gain_value_snapshot = r["gain"]           # (a Shift let go of after the switch saves only a real change)
    _log("switched here from stereopsis: %s" % (r,))


def _clear_own_markers(ey):
    """a switch is not a crash: the modes' crash markers this process armed (.native_trial holding our PID) go"""
    import glob
    me = str(os.getpid())
    for p in glob.glob(os.path.join(glob.escape(ey.MODES_PATH), "*", ".native_trial")):
        try:
            if open(p).read().strip() == me:
                os.remove(p)
        except Exception:
            pass


def _swap_to_fork(ey, root, why):
    """hand this (stock) engine process to stereopsis, on the mode on screen. Returns only if it cannot."""
    if not _swap_enabled():
        _log("not switching to stereopsis (%s): %s" % (why, "SWAP = 0" if os.path.exists(FORK_MAIN)
                                                        else FORK_MAIN + " missing"))
        return
    _log("switching to stereopsis (%s) on %s" % (why, ey.mode))
    try:
        _write_atomic(os.path.join(FORK_DIR, "RETURN"), json.dumps({
            "from": "stock", "mode": str(ey.mode), "persist": not bool(ey.auto_clear), "fg": int(ey.fg_palette),
            "bg": int(ey.bg_palette), "osd": bool(ey.show_osd),
            "gain": float(ey.config.get("audio_gain", 0.25))}) + "\n")
        _write_atomic(os.path.join(root, "STEREOPSIS"), "boot\n")
    except Exception as e:
        _log("could not write the switch files (%r): staying on the stock engine" % (e,))
        try:
            os.remove(os.path.join(FORK_DIR, "RETURN"))
        except Exception:
            pass
        return
    _clear_own_markers(ey)
    try:
        sys.modules["osd"].loading_banner(sys.modules["__main__"].hwscreen, "Switching to stereopsis")
    except Exception:
        pass
    _exec_fork(root, "switch: " + why)


def _stock_state(ey, controls):
    def pal(i):
        try:
            return str(ey.palettes[i]["name"])
        except Exception:
            return str(i)
    main = sys.modules.get("__main__")
    return {"engine": "stock", "version": "stock", "ready": bool(main is not None and hasattr(main, "start")),
            "mode": str(ey.mode), "mode_index": int(ey.mode_index), "mode_count": len(ey.mode_names),
            "fps": float(getattr(ey, "fps", 0) or 0), "persist": not bool(ey.auto_clear), "osd": bool(ey.show_osd),
            "fg": pal(ey.fg_palette), "bg": pal(ey.bg_palette), "fg_index": int(ey.fg_palette),
            "bg_index": int(ey.bg_palette), "gain": float(ey.config.get("audio_gain", 0.0)),
            "controls": bool(controls), "swap": bool(controls) and _swap_enabled()}


STOCK_PAGE = r"""<!doctype html>
<html lang="en"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>EYESY &middot; stock engine</title>
<style>
:root { color-scheme: dark; --bg: #0b0b0d; --ctl: #212128; --ctl2: #2d2d36; --line: #33333d; --text: #e8e8ec;
  --dim: #9a9aa6; --held: #f2b35e; --accent: #8ab4f8; }
* { box-sizing: border-box; }
html, body { margin: 0; background: var(--bg); color: var(--text); font: 15px/1.45 system-ui, -apple-system, "Segoe UI", sans-serif; }
main { max-width: 540px; margin: 0 auto; padding: 28px 16px; display: grid; gap: 16px; }
h1 { font-size: 17px; margin: 0; font-weight: 600; }
h1 span { color: var(--dim); font-weight: 400; }
#mode { font-size: 22px; font-weight: 600; overflow-wrap: anywhere; }
#meta, .hint { color: var(--dim); }
.hint { font-size: 13px; }
button { font: inherit; color: var(--text); background: var(--ctl); border: 1px solid var(--line); border-radius: 8px;
  min-height: 48px; padding: 0 18px; cursor: pointer; justify-self: start; touch-action: manipulation; }
button:active { background: var(--ctl2); }
button.armed { border-color: var(--held); color: var(--held); }
#msg { color: var(--accent); min-height: 1.45em; }
</style></head>
<body><main>
<h1>EYESY <span>&middot; the stock engine</span></h1>
<div><div id="mode">connecting&hellip;</div><div id="meta"></div></div>
<button id="swap" hidden>Switch to stereopsis</button>
<div id="msg" role="status"></div>
<p class="hint">Critter &amp; Guitari's original engine is running, on the mode stereopsis showed. There is no live
picture or remote here: stereopsis brings them back. On the EYESY: hold Shift + Persist for a second.</p>
</main>
<script>
"use strict";
const $ = (id) => document.getElementById(id);
const btn = $("swap"), LABEL = "Switch to stereopsis";
let armed = -1e9, swapping = false, swapAt = 0;
btn.addEventListener("click", async () => {
  if (performance.now() - armed > 4000) {
    armed = performance.now();
    btn.classList.add("armed");
    btn.textContent = "Click again to switch";
    setTimeout(() => { if (performance.now() - armed >= 4000) { btn.classList.remove("armed"); btn.textContent = LABEL; } }, 4100);
    return;
  }
  armed = -1e9;
  btn.classList.remove("armed");
  btn.textContent = LABEL;
  swapping = true;
  swapAt = performance.now();
  $("msg").textContent = "Switching to stereopsis… this page follows in about 10 s";
  try {
    const r = await fetch("/control", { method: "POST", headers: { "Content-Type": "application/json" },
      body: JSON.stringify([{ act: "swap" }]) });
    if (!r.ok) {
      swapping = false;
      let why = "";
      try { why = (await r.json()).error || ""; } catch (e) {}
      $("msg").textContent = "Not switched: " + (why || "HTTP " + r.status);
    }
  } catch (e) {}
});
async function poll() {
  try {
    const s = await (await fetch("/state.json", { cache: "no-store" })).json();
    if (s.engine !== "stock") { location.reload(); return; }           // stereopsis answers here again
    if (swapping && performance.now() - swapAt > 12000) {             // still here: it did not switch
      swapping = false;
      $("msg").textContent = "Not switched: the stock engine is still running (the EYESY's log says why)";
    }
    if (!s.ready) {
      $("mode").textContent = "starting the stock engine…";
      $("meta").textContent = "it loads every mode first (about 15 s)";
    } else {
      $("mode").textContent = s.mode;
      $("meta").textContent = [Math.round(s.fps) + " fps (the stock engine caps at 30)", s.persist ? "Persist on" : "",
        s.osd ? "OSD on" : "", "colour " + s.fg, "background " + s.bg].filter(Boolean).join(" · ");
    }
    btn.hidden = !s.swap || !s.ready;
  } catch (e) {
    $("mode").textContent = swapping ? "switching engines…" : "no connection";
    $("meta").textContent = "";
    btn.hidden = true;
  }
  setTimeout(poll, 1000);
}
poll();
</script></body></html>
"""


def _serve_page(ey, W):
    """the small page (and /state.json, /control) on stereopsis's preview port, while the stock engine runs"""
    if _fork_flag("PREVIEW", "1") == "0":
        return
    try:
        port = int(_fork_flag("PREVIEW_PORT", "8081") or 8081)
    except Exception:
        port = 8081
    controls = _fork_flag("CONTROLS", "1") != "0"
    from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

    class Handler(BaseHTTPRequestHandler):
        protocol_version = "HTTP/1.1"
        timeout = 30

        def log_message(self, fmt, *args):
            pass

        def _send(self, code, ctype, body):
            self.send_response(code)
            self.send_header("Content-Type", ctype)
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(body)

        def _json(self, code, obj):
            self._send(code, "application/json", json.dumps(obj).encode())

        def do_GET(self):
            path = self.path.split("?", 1)[0]
            try:
                if path in ("/", "/index.html"):
                    self._send(200, "text/html; charset=utf-8", STOCK_PAGE.encode())
                elif path == "/state.json":
                    self._json(200, _stock_state(ey, controls))
                else:
                    self._send(404, "text/plain", b"not here: this is the stock engine\n")
            except OSError:
                self.close_connection = True
            except Exception as e:
                self.close_connection = True
                _log("page: GET %s failed: %r" % (path[:80], e))

        def do_POST(self):
            try:
                try:
                    n = int(self.headers.get("Content-Length") or -1)
                except ValueError:
                    n = -1
                if n < 0 or n > 4096:
                    self.close_connection = True
                    return self._json(413 if n > 4096 else 411, {"ok": False, "error": "body length"})
                body = self.rfile.read(n) if n else b""
                if self.path.split("?", 1)[0] != "/control":
                    return self._send(404, "text/plain", b"not found\n")
                if not controls:
                    return self._json(403, {"ok": False, "error": "this EYESY's page is view-only (CONTROLS=0)"})
                if self.headers.get("Content-Type", "").split(";")[0].strip().lower() != "application/json":
                    return self._json(415, {"ok": False, "error": "send JSON (Content-Type: application/json)"})
                origin = self.headers.get("Origin")
                if origin is not None and origin != "http://" + self.headers.get("Host", ""):
                    return self._json(403, {"ok": False, "error": "controls only take requests from this page"})
                try:
                    msg = json.loads(body.decode("utf-8"))
                except Exception:
                    return self._json(400, {"ok": False, "error": "bad JSON"})
                acts = msg if isinstance(msg, list) else [msg]
                if len(acts) != 1 or not isinstance(acts[0], dict) or acts[0].get("act") != "swap":
                    return self._json(400, {"ok": False, "error": "the stock engine only takes {\"act\": \"swap\"}"})
                if not _swap_enabled():
                    return self._json(400, {"ok": False, "error": "switching engines is off on this EYESY (SWAP = 0)"})
                W["ask"] = "the page"
                self._json(200, {"ok": True, "queued": 1})
            except OSError:
                self.close_connection = True
            except Exception as e:
                self.close_connection = True
                _log("page: POST failed: %r" % (e,))

    class Server(ThreadingHTTPServer):
        daemon_threads = True
        allow_reuse_address = True

    try:
        srv = Server(("0.0.0.0", port), Handler)
    except OSError as e:
        _log("switching engines: page port %d unavailable (%s)" % (port, e))
        return
    threading.Thread(target=srv.serve_forever, name="lab-page", daemon=True).start()


def _swap_install(root):
    """set up switching to stereopsis inside the stock engine (once per process)"""
    if not os.path.exists(FORK_MAIN):
        return
    main = sys.modules.get("__main__")
    ey = getattr(main, "eyesy", None)
    if ey is None or not callable(getattr(ey, "dispatch_key_event", None)) or \
            not callable(getattr(ey, "update_key_repeater", None)):
        return
    L = _lab()
    if L.get("swap") is not None:
        return
    W = L["swap"] = {"t0": None, "ask": None, "arrive": True}
    orig_key, orig_frame = ey.dispatch_key_event, ey.update_key_repeater

    def key_event(k, v):
        try:
            if k == 3 and v > 0 and ey.key2_status and not ey.menu_mode:
                W["t0"] = time.monotonic()
            elif k in (2, 3) and v <= 0:
                W["t0"] = None
        except Exception:
            pass
        return orig_key(k, v)

    def every_frame(*a, **k):
        r = orig_frame(*a, **k)
        try:
            if W["arrive"]:                           # the first frame: open what stereopsis left
                W["arrive"] = False
                ret = _read_return(ey)
                if ret:
                    _apply_return(ey, ret)
            if W["t0"] is not None and not ey.menu_mode and time.monotonic() - W["t0"] >= SWAP_HOLD_S:
                W["t0"] = None
                W["ask"] = "Shift + Persist"
            if W["ask"]:
                why, W["ask"] = W["ask"], None
                _swap_to_fork(ey, root, why)
        except Exception as e:
            _log("switching engines: %r" % (e,))
        return r

    ey.dispatch_key_event = key_event
    ey.update_key_repeater = every_frame
    _serve_page(ey, W)
    _log("switching engines: hold Shift + Persist for a second, or the page on port %s"
         % (_fork_flag("PREVIEW_PORT", "8081") or 8081))


# ---------------------------------------------------------------------------------------------- the flip
def _lab_flip(*a, **k):
    """Replaces pygame.display.flip for the whole engine (installed once, kept until the engine restarts)."""
    L = pygame._eyesy_lab
    t0 = time.perf_counter()
    done = False
    if L["turbo"] and L.get("scale"):
        try:
            small = sys.modules["__main__"].mode_screen
            rc = L["lib"].tb_plane_present(small._pixels_address, small.get_pitch(), small.get_width(), small.get_height())
        except Exception as e:
            rc = -100000
            _log("tb_plane_present raised %r" % (e,))
        if rc == 0:
            done = True
        else:
            _scale_off(L, "tb_plane_present failed (%d)" % rc, failed=True)
    if L["turbo"] and not done:
        try:
            surf = pygame.display.get_surface()
            rc = L["lib"].tb_present(surf._pixels_address, surf.get_pitch(), surf.get_width(), surf.get_height())
        except Exception as e:
            rc = -100000
            _log("tb_present raised %r" % (e,))
        if rc == 0:
            done = True
            L["turbo_frames"] += 1
            if L["turbo_frames"] == TRIAL_FRAMES:
                _trial_clear(L)
        else:
            _turbo_off(L, "tb_present failed (%d)" % rc, failed=True)
    if not done:
        L["orig_flip"](*a, **k)
    if L["meter"]:
        _meter(L, t0, time.perf_counter(), done)


_lab_flip._eyesy_lab = True


def _install_flip(L):
    cur = pygame.display.flip
    if cur is _lab_flip:
        return
    if getattr(cur, "_eyesy_lab", False):          # an older copy of this module installed it: take over
        pygame.display.flip = _lab_flip
        return
    L["orig_flip"] = cur
    pygame.display.flip = _lab_flip


class _LabClock(object):
    """Stands in for the engine's pygame Clock while TURBO is on: the stock loop calls tick(30)."""
    _eyesy_lab = True

    def __init__(self, clk):
        self._c = clk

    def tick(self, framerate=0):
        L = pygame._eyesy_lab
        if L["turbo"]:
            cap = L["fpscap"]
            # at 60+ the vblank wait inside tb_present paces the loop; tick(0) only keeps the clock's books
            return self._c.tick(0 if cap >= 60 else cap)
        return self._c.tick(framerate)

    def __getattr__(self, name):
        return getattr(self._c, name)


def _clock_install(L):
    main = sys.modules.get("__main__")
    clk = getattr(main, "clocker", None)
    if clk is not None and hasattr(clk, "tick") and not getattr(clk, "_eyesy_lab", False):
        L["orig_clock"] = clk
        main.clocker = _LabClock(clk)


def _clock_restore(L):
    main = sys.modules.get("__main__")
    if main is not None and getattr(getattr(main, "clocker", None), "_eyesy_lab", False) and L["orig_clock"] is not None:
        main.clocker = L["orig_clock"]


def _master_fd(lib):
    try:
        names = os.listdir("/proc/self/fd")
    except Exception:
        return -1
    for n in names:
        try:
            if os.readlink("/proc/self/fd/" + n).startswith("/dev/dri/card") and lib.tb_is_master(int(n)) == 1:
                return int(n)
        except Exception:
            continue
    return -1


def _turbo_on(L):
    lib = L["lib"]
    fd = _master_fd(lib)
    if fd < 0:
        L["failed"] = "no DRM master fd in this process (not on KMS?)"
        _log("TURBO not started: " + L["failed"])
        return
    disp = pygame.display.get_surface()
    if disp is None or disp.get_bitsize() != 32 or disp.get_masks()[:3] != (0xFF0000, 0xFF00, 0xFF):
        L["failed"] = "display surface is not XRGB8888"
        _log("TURBO not started: " + L["failed"])
        return
    _trial_arm(L)
    rc = lib.tb_init(fd, disp.get_width(), disp.get_height(), 3)
    if rc != 0:
        _trial_clear(L)
        L["failed"] = "tb_init returned %d" % rc
        _log("TURBO not started: " + L["failed"])
        return
    L["fd"] = fd
    L["turbo_frames"] = 0
    L["prev_stats"] = None
    L["turbo"] = True
    _clock_install(L)
    st = _stats(L)
    _log("TURBO on: fd %d, crtc %d (index %d), %dx%d, scanout pitch %d, frame cap %d"
         % (fd, st[9], st[10], st[11], st[12], st[13], L["fpscap"]))


def _turbo_off(L, why, failed=False):
    if L.get("scale"):
        _scale_off(L, "TURBO going off")
    L["turbo"] = False
    rc = L["lib"].tb_handback() if L["lib"] is not None else 0
    L["retired"] = True
    _clock_restore(L)
    _trial_clear(L)
    if failed:
        L["failed"] = why
        _write(L["root"], "TURBO", "0\n")
    _log("TURBO off (%s), SDL presents again (handback %d); restart the engine to use TURBO again" % (why, rc))


def _stats(L):
    out = (ctypes.c_double * 16)()
    L["lib"].tb_stats(out, 16)
    return list(out)


# ---------------------------------------------------------------------------------------------- render scale
def _scale_parse(txt):
    m = re.match(r"^\s*(\d{2,4})\s*x\s*(\d{2,4})\s*$", txt or "")
    if not m:
        return None
    w, h = int(m.group(1)), int(m.group(2))
    return (w, h) if 16 <= w <= 2048 and 16 <= h <= 2048 else None


def _resetup_check(L, ey):
    """Runs every frame after the engine read its inputs (so after any mode change): a mode whose last
    setup() was at a different size than it is about to draw at gets setup() again, in this same frame."""
    size = (ey.xres, ey.yres)
    ss = L.setdefault("setup_size", {})
    if getattr(ey, "run_setup", False):              # the engine re-runs setup anyway (editor reload)
        ss[ey.mode] = size
    elif ss.get(ey.mode, L.get("full_size", size)) != size:
        ey.run_setup = True
        ss[ey.mode] = size


def _hook_install(L):
    ey = L["eyesy"]
    base = type(ey).update_knobs_and_notes              # always wrap the class method, never a wrapper

    def hook():
        base(ey)
        try:
            _resetup_check(pygame._eyesy_lab, ey)
        except Exception as e:
            _log("resetup hook: %r" % (e,))
    hook._eyesy_lab_hook = True
    ey.update_knobs_and_notes = hook


def _plane_stats(L):
    out = (ctypes.c_double * 10)()
    L["lib"].tb_plane_stats(out, 10)
    return list(out)


def _scale_on(L, w, h):
    main = sys.modules.get("__main__")
    ey = L["eyesy"]
    big = getattr(main, "mode_screen", None)
    if big is None:
        L["scale_failed"] = "no engine mode_screen"
        return
    rc = L["lib"].tb_plane_init(w, h, 0, 0, big.get_width(), big.get_height())
    if rc <= 0:
        L["scale_failed"] = "tb_plane_init returned %d" % rc
        _log("SCALE not started: " + L["scale_failed"])
        return
    small = pygame.Surface((w, h), 0, 32, (0xFF0000, 0xFF00, 0xFF, 0))
    L["full_size"] = (ey.xres, ey.yres)
    L["scale"] = {"w": w, "h": h, "big": big, "small": small, "screen": getattr(ey, "screen", None), "plane": rc}
    L["prev_pstats"] = _plane_stats(L)
    _hook_install(L)
    main.mode_screen = small
    ey.screen = small
    ey.xres, ey.yres = w, h
    _log("SCALE on: every mode renders %dx%d, overlay plane %d scales it to %dx%d"
         % (w, h, rc, big.get_width(), big.get_height()))


def _scale_off(L, why, failed=False):
    sc = L.get("scale")
    if not sc:
        return
    rc = L["lib"].tb_plane_off()
    main = sys.modules.get("__main__")
    ey = L["eyesy"]
    main.mode_screen = sc["big"]
    if sc["screen"] is not None:
        ey.screen = sc["screen"]
    ey.xres, ey.yres = L["full_size"]
    L["scale"] = None
    if failed:
        L["scale_failed"] = why
        _write(L["root"], "SCALE", "0\n")
    _log("SCALE off (%s), plane off %d; modes get set up at full size again as they come up" % (why, rc))


# ---------------------------------------------------------------------------------------------- meter
def _meter(L, t0, t1, turbo):
    ey = L["eyesy"]
    mode = getattr(ey, "mode", "?")
    M = L["m"]
    if M.get("mode") != mode:
        if M.get("mode") is not None and len(M.get("iv", [])) >= 10:
            _meter_log(L, M)
        L["m"] = M = {"mode": mode, "iv": [], "pr": [], "last": None, "since": t0, "turbo": turbo}
        L["prev_stats"] = _stats(L) if (turbo and L["lib"] is not None) else None
        if L.get("scale") and L["lib"] is not None:
            L["prev_pstats"] = _plane_stats(L)
    if M["last"] is not None:
        M["iv"].append(t0 - M["last"])
    M["last"] = t0
    M["pr"].append(t1 - t0)
    M["turbo"] = turbo
    if len(M["iv"]) > 1200:
        del M["iv"][:600]
        del M["pr"][:600]
    if t1 - M["since"] >= METER_EVERY and len(M["iv"]) >= 10:
        _meter_log(L, M)
        M.update(iv=[], pr=[], since=t1)


def _pct(xs, p):
    s = sorted(xs)
    return s[min(len(s) - 1, int(len(s) * p))] if s else 0.0


def _meter_log(L, M):
    iv, pr = M["iv"], M["pr"]
    fps = len(iv) / sum(iv) if iv and sum(iv) > 0 else 0.0
    extra = ""
    sc = L.get("scale")
    if sc and L["lib"] is not None:
        st = _plane_stats(L)
        prev = L.get("prev_pstats") or st
        n = max(1.0, st[0] - prev[0])
        c = max(1.0, st[1] - prev[1])
        extra = " | %dx%d on plane: copy %.2f wait %.2f commit %.2f ms, err %d" % (
            sc["w"], sc["h"], (st[3] - prev[3]) / n, (st[4] - prev[4]) / n, (st[5] - prev[5]) / c, st[2] - prev[2])
        L["prev_pstats"] = st
    elif M["turbo"] and L["lib"] is not None:
        st = _stats(L)
        prev = L.get("prev_stats") or st
        n = max(1.0, st[0] - prev[0])
        extra = " | copy %.2f wait %.2f flip %.2f ms, ebusy %d, err %d" % (
            (st[5] - prev[5]) / n, (st[6] - prev[6]) / n, (st[7] - prev[7]) / n, st[2] - prev[2], st[4] - prev[4])
        L["prev_stats"] = st
    line = "METER %s | %.1f fps | frame p50 %.1f p90 %.1f max %.1f ms | present p50 %.2f ms | %s%s" % (
        M["mode"], fps, 1000 * _pct(iv, 0.5), 1000 * _pct(iv, 0.9), 1000 * max(iv), 1000 * _pct(pr, 0.5),
        ("SCALE" if sc else "TURBO") if M["turbo"] else "stock", extra)
    L["last_meter"] = line
    _log(line)


# ---------------------------------------------------------------------------------------------- probe
def _probe(L):
    lib = L["lib"]
    try:
        _log("PROBE kernel: " + open("/proc/version").read().strip()[:160])
    except Exception:
        pass
    _log("PROBE python %s, pygame %s, SDL %s, display driver %s" % (
        platform.python_version(), pygame.version.ver, ".".join(str(v) for v in pygame.get_sdl_version()),
        pygame.display.get_driver()))
    fds = []
    try:
        for n in os.listdir("/proc/self/fd"):
            try:
                t = os.readlink("/proc/self/fd/" + n)
            except Exception:
                continue
            if t.startswith("/dev/dri/"):
                fds.append((int(n), t, lib.tb_is_master(int(n)) if t.startswith("/dev/dri/card") else None))
    except Exception as e:
        _log("PROBE cannot list /proc/self/fd: %r" % (e,))
    _log("PROBE DRM fds (fd, node, master): %s" % (fds,))
    fd = _master_fd(lib)
    if fd < 0:
        _log("PROBE no DRM master fd - nothing more to probe")
        return
    buf = ctypes.create_string_buffer(16384)
    rc = lib.tb_probe(fd, buf, 16384)
    for line in buf.value.decode("utf-8", "replace").splitlines():
        _log("PROBE " + line)
    _log("PROBE done (%d)" % rc)


# ---------------------------------------------------------------------------------------------- mode
def setup(screen, eyesy):
    S.clear()
    root = eyesy.mode_root if getattr(eyesy, "mode_root", "") else os.path.dirname(os.path.abspath(__file__))
    root = root.rstrip("/")
    L = _lab()
    L["root"] = root
    L["eyesy"] = eyesy
    S["root"] = root
    S["sw"] = {k: _flag(root, k) for k in ("METER", "PROBE", "TURBO")}
    S["fork"] = _fork_mode(root)
    try:
        S["scale"] = _scale_parse(open(os.path.join(root, "SCALE")).read())
    except Exception:
        S["scale"] = None
    L["fpscap"] = int(max(10, min(120, _num(root, "FPSCAP", 60))))
    S["frame"] = 0
    S["probed"] = False
    S["text_at"] = -1.0
    S["text"] = []
    _trial_check(root)
    S["sw"]["TURBO"] = _flag(root, "TURBO")         # re-read: the trial check may have switched it off
    if UNDER_FORK:
        L["native"] = "not needed under stereopsis"
    elif L["lib"] is None and (L["native"] == "not built" or str(L["native"]).startswith("off")):
        L["native"] = "compiling"
        threading.Thread(target=_compile_and_load, args=(root, L), daemon=True).start()


def _apply(L):
    if UNDER_FORK:
        return                                      # stereopsis has its own display path and METER
    sw = S["sw"]
    if S.get("fork") == "1" and not S.get("fork_tried"):
        S["fork_tried"] = True
        if _fork_guard(S["root"]):
            _exec_fork(S["root"], "STEREOPSIS=1, lab drawn")
    if sw["METER"] or sw["TURBO"]:
        _install_flip(L)
    L["meter"] = sw["METER"]
    if L["lib"] is None:
        return
    if sw["PROBE"] and not S["probed"]:
        S["probed"] = True
        _probe(L)
    if sw["TURBO"] and not L["turbo"] and not L["retired"] and not L["failed"]:
        _turbo_on(L)
    elif not sw["TURBO"] and L["turbo"]:
        _turbo_off(L, "switched off")
    want = S["scale"] if L["turbo"] else None
    cur = (L["scale"]["w"], L["scale"]["h"]) if L.get("scale") else None
    if cur and want != cur:
        _scale_off(L, "switched off" if not want else "size changed")
        cur = None
    if want and not cur and not L.get("scale_failed"):
        _scale_on(L, *want)


def draw(screen, eyesy):
    L = _lab()
    S["frame"] += 1
    if S["frame"] == 1 or S["frame"] % 15 == 0:
        _apply(L)
    xr, yr = screen.get_width(), screen.get_height()
    now = time.time()
    if now - S["text_at"] >= 1.0:                    # re-render the text once a second, blit it every frame
        S["text_at"] = now
        font = S.get("font") or pygame.font.Font(None, max(18, yr // 26))
        S["font"] = font
        path = "TURBO: KMS page flip, 3 buffers" if L["turbo"] else "stock: SDL uploads a GL texture every frame"
        if L.get("scale"):
            path = "SCALE: every mode renders %dx%d, the display hardware scales it to full screen" % (
                L["scale"]["w"], L["scale"]["h"])
        if L["retired"] and not L["turbo"]:
            path += " (TURBO used this session; restart the engine to use it again)"
        if UNDER_FORK:
            path = "%s: compositor, atomic KMS commits, no SDL video" % getattr(eyesy, "VERSION", "?")
        lines = ["ENGINE LAB", "display path: " + path,
                 "engine fps: %.1f   frame cap: %s" % (getattr(eyesy, "fps", 0.0), "stereopsis FPSCAP" if UNDER_FORK
                                                              else (L["fpscap"] if L["turbo"] else 30)),
                 "switches: METER %d  PROBE %d  TURBO %d    turbo.c: %s" % (
                     S["sw"]["METER"], S["sw"]["PROBE"], S["sw"]["TURBO"], L["native"]) + "   STEREOPSIS %s" % S.get("fork", "0")]
        if L["failed"]:
            lines.append("TURBO failed: " + L["failed"])
        if L.get("scale_failed"):
            lines.append("SCALE failed: " + L["scale_failed"])
        if L["last_meter"]:
            lines.append(L["last_meter"][:110])
        lines.append("an engine restart always returns the stock display path")
        S["text"] = [font.render(t, True, (230, 230, 230) if i else (120, 220, 255)) for i, t in enumerate(lines)]
    y = yr // 12
    for surf in S["text"]:
        screen.blit(surf, (xr // 20, y))
        y += surf.get_height() + 6
    # something that changes every frame, so a capture can count delivered frames
    x = (S["frame"] * 12) % max(1, xr - 40)
    pygame.draw.rect(screen, (255, 120, 40) if L["turbo"] else (80, 160, 255), (x, yr - yr // 8, 40, yr // 16))


_boot_hook()
