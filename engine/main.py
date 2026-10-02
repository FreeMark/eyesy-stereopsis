# stereopsis - a faster engine for the EYESY.
# Based on Critter & Guitari's EYESY_OS engines/python/main.py (BSD 3-Clause, see LICENSE.txt); every
# change from theirs is marked "FORK:". The mode API is unchanged, so every existing mode runs as is.
#
# FORK: what is different
#   * the display is ours (display.py + kms.c). SDL runs on its "dummy" video driver, so the ~17 ms per
#     frame SDL spent uploading each frame as a GL texture is gone. A compositor thread in kms.c puts each
#     frame on screen with one atomic commit: the frame is copied into scanout memory and cleared for the
#     next frame on another CPU core while the engine already draws the next one (two surfaces, taking
#     turns; persist mode keeps one)
#   * per-mode render size (/sdcard/stereopsis/scale.json): a mode can draw at e.g. 640x360 and the
#     display hardware scales it to full screen for free; setup() runs again at the size a mode draws at
#   * the OSD is drawn on its own transparent layer on top, not copied into the frame
#   * a browser preview at http://<the EYESY's address>:8081/ (preview.py; frames made by the compositor
#     on spare cores, only while someone watches); /sdcard/stereopsis/PREVIEW = 0 turns it off
#   * the preview page is also a remote: knobs, mode, scene, palettes, OSD, Persist, Trigger, grab, gain.
#     Its actions are queued and applied by this loop, once per frame; a knob set there is held like a MIDI
#     CC until that knob on the EYESY moves. /sdcard/stereopsis/CONTROLS = 0 makes the page view-only
#   * audio analysis for modes (analysis.py; audio.c runs in the audio process on the full 32 kHz stream):
#     eyesy.audio_level/_bass/_mid/_treble, audio_beat, beat, bpm, audio_bands, audio_fft, audio_wave.
#     /sdcard/stereopsis/TRIGGER = beat (or both) lets a detected kick fire eyesy.trig (stock: loudness)
#   * OSC is polled without the 1 ms blocking timeout; SDL audio playback is not opened
#   * frame cap from /sdcard/stereopsis/FPSCAP (default 60, paced by the display) instead of 30
#   * /sdcard/stereopsis/METER = 1 logs each mode's real frame rate (same format as the Engine Lab's METER)
#   * /sdcard/stereopsis/LAZY = 1: each mode's setup() runs when it is first shown instead of at boot
#   * modes keep stock's speed at 60 fps (stock draws at 30 and nearly every mode moves per frame): eyesy.dt / step /
#     tick for the modes (eyesy.py; the factory modes are ported to them in ../modes), the colour LFO and Python's
#     random replayed on the frames between 30 fps frames, the loudness trigger at most once per 30 fps frame;
#     "@fps" in scale.json runs a mode at a lower rate, locked to the display (a swap interval in kms.c)
#   * .trial crash marker, cleared after 300 good frames (the Engine Lab stops launching a build
#     that died before that)
#   * switching engines: Shift + Persist held for a second (or the page's button) switches to the stock engine on the
#     same mode, and the Engine Lab switches back the same way (swap_to_stock below); SWAP = 0 turns it off
#   * frame scheduling for audio-to-picture latency (Scheduler below, 0.8): a frame starts just in time for its vblank
#     when it fits (its audio ~12-17 ms old on screen instead of ~50); /sdcard/stereopsis/JIT = 0 turns it off.
#     /sdcard/stereopsis/LATENCY = 1 logs how old the audio is when each frame reaches the screen (Latency below)
#   * the page's Size buttons (0.9): the mode on screen draws at the display's size, half or a quarter of it, from the
#     next frame, saved in scale.json (set_render_size below)
#   * eyesy.trig_audio (0.9.1): True when this frame's trig came only from the audio, so modes can tell a performer's
#     Trigger (button, MIDI, the page) from the loudness trigger
import os
os.environ["SDL_VIDEODRIVER"] = "dummy"     # FORK: SDL never touches the screen (display.py does)
os.environ["SDL_AUDIODRIVER"] = "dummy"     # FORK: pygame.init() must not open ALSA playback
os.environ["STEREOPSIS"] = "1"             # FORK: modes (the Engine Lab) can tell they run on stereopsis
from multiprocessing import Process, Array, Value, Lock
from ctypes import c_float
import collections
import json
import time
import sys
import psutil
import math
import random
import traceback
import liblo
import pygame
from pygame.locals import *
import midi
import eyesy
import osc
import sound
import osd
import usbdrive
import display
import analysis                             # FORK: audio analysis for modes (audio.c in the audio process)
from screen_main_menu import ScreenMainMenu
from screen_test import ScreenTest
from screen_video_settings import ScreenVideoSettings
from screen_palette import ScreenPalette
from screen_wifi import ScreenWiFi
from screen_applogs import ScreenApplogs
from screen_midi_settings import ScreenMIDISettings
from screen_midi_pc_mapping import ScreenMIDIPCMapping
from screen_flash_drive import ScreenFlashDrive

FORK_DIR = os.environ.get("STEREOPSIS_DIR", "/sdcard/stereopsis")    # FORK: switches + crash marker live here
TRIAL_FRAMES = 300
STEREOPSIS_VERSION = "0.9.1"


def fork_flag(name, default=None):
    try:
        return open(os.path.join(FORK_DIR, name)).read().strip()
    except Exception:
        return default


def exitexit(code):
    print("EXIT exiting\n")
    pygame.display.quit()
    pygame.quit()
    print("stopping audio process")
    if audio_process.is_alive():  # Check if the process is still running
        audio_process.terminate()  # Terminate the process
        audio_process.join()       # Ensure the process has fully terminated
    print("closing audio")
    audio_process.close()  # Now it's safe to close the process
    print("closing midi")
    midi.close()
    print("closing osc")
    osc.close()
    print("exiting...")
    sys.exit(code)


# FORK: render size (and optionally frame rate) per mode. scale.json = {"default": "full", "modes":
# {"T - Woven Feedback": "640x360", "S - Boids": "full@20"}}: "size", "size@fps" or "@fps"
def parse_size(v, full):
    v = str(v or "").split("@")[0].strip().lower()
    if not v or v == "full":
        return full
    try:
        w, h = (int(x) for x in v.split("x"))
    except Exception:
        return full
    if w < 64 or h < 36 or w > full[0] or h > full[1]:
        return full
    return (w, h)


def parse_rate(v):
    """the "@fps" part of a scale.json value, or None"""
    v = str(v or "")
    if "@" not in v:
        return None
    try:
        r = float(v.split("@", 1)[1])
    except Exception:
        return None
    return r if 5 <= r <= 120 else None


def load_scale_config(full):
    path = os.path.join(FORK_DIR, "scale.json")
    try:
        cfg = json.load(open(path))
    except FileNotFoundError:
        cfg = {}
    except Exception as e:
        print("[stereopsis] scale.json unreadable (%r): every mode at full size" % (e,), flush=True)
        cfg = {}
    default = parse_size(cfg.get("default"), full)
    modes = {str(k): parse_size(v, full) for k, v in (cfg.get("modes") or {}).items()}
    rates = {str(k): parse_rate(v) for k, v in (cfg.get("modes") or {}).items() if parse_rate(v)}
    return default, modes, rates


# FORK: the page's Size buttons (0.9): the mode on screen draws at the display's size, half or a quarter of it. In
# effect at the next frame (the mode is set up again at that size) and saved in scale.json, keeping an "@fps"
RENDER_PRESETS = {"full": 1, "half": 2, "quarter": 4}


def set_render_size(mode_name, preset):
    d = RENDER_PRESETS[preset]
    size = (FULL[0] // d, FULL[1] // d)
    scale_modes[mode_name] = size
    if sched is not None:
        sched.forget()                               # its frames take a different time now: measure them again
    path = os.path.join(FORK_DIR, "scale.json")
    try:
        try:
            with open(path) as f:
                cfg = json.load(f)
        except FileNotFoundError:
            cfg = {}
        if not isinstance(cfg, dict) or not isinstance(cfg.get("modes", {}), dict):
            raise ValueError('not {"default": ..., "modes": {...}}')
        modes = cfg.setdefault("modes", {})
        old = str(modes.get(mode_name, ""))
        rate = old.split("@", 1)[1] if "@" in old else ""
        if size == scale_default and not rate:
            modes.pop(mode_name, None)
        else:
            modes[mode_name] = ("full" if size == FULL else "%dx%d" % size) + ("@" + rate if rate else "")
        cfg.setdefault("default", "full")
        write_atomic(path, json.dumps(cfg, indent=2) + "\n")
        print("[stereopsis] %s now draws at %dx%d (the page's Size; saved in scale.json)" % (mode_name, size[0], size[1]),
              flush=True)
    except Exception as e:
        print("[stereopsis] %s draws at %dx%d until the next restart (scale.json not saved: %r)" % (
            mode_name, size[0], size[1], e), flush=True)


# FORK: per-mode frame meter (same line format as the Engine Lab, so tools/turbo_ab.py reads both)
class Meter(object):
    EVERY = 2.0

    def __init__(self, disp):
        self.disp = disp
        self.mode = None
        self.reset(time.perf_counter())

    def stats(self):
        return self.disp.comp_stats() if self.disp.comp else self.disp.stats()

    def reset(self, now):
        self.iv, self.pr, self.last, self.since = [], [], None, now
        self.prev = self.stats()

    def tick(self, mode, t0, t1, path):
        if mode != self.mode:
            if self.mode is not None and len(self.iv) >= 10:
                self.log(path)
            self.mode = mode
            self.reset(t0)
        if self.last is not None:
            self.iv.append(t0 - self.last)
        self.last = t0
        self.pr.append(t1 - t0)
        if t1 - self.since >= self.EVERY and len(self.iv) >= 10:
            self.log(path)
            self.iv, self.pr, self.since = [], [], t1

    def log(self, path):
        iv, pr = sorted(self.iv), sorted(self.pr)
        st = self.stats()
        n = max(1.0, st[0] - self.prev[0])
        if self.disp.comp:
            extra = " | worker: copy %.2f clear %.2f osd %.2f flipwait %.2f ms, engine waited %.2f ms, err %d" % (
                (st[4] - self.prev[4]) / n, (st[5] - self.prev[5]) / n, (st[6] - self.prev[6]) / n,
                (st[7] - self.prev[7]) / n, (st[9] - self.prev[9]) / n, st[3] - self.prev[3])
        else:
            extra = " | copy %.2f wait %.2f flip %.2f ms, ebusy %d, err %d" % (
                (st[5] - self.prev[5]) / n, (st[6] - self.prev[6]) / n, (st[7] - self.prev[7]) / n,
                st[2] - self.prev[2], st[4] - self.prev[4])
        self.prev = st
        print("[stereopsis] METER %s | %.1f fps | frame p50 %.1f p90 %.1f max %.1f ms | present p50 %.2f ms | %s%s" % (
            self.mode, len(iv) / sum(iv), 1000 * iv[len(iv) // 2], 1000 * iv[int(len(iv) * 0.9)], 1000 * iv[-1],
            1000 * pr[len(pr) // 2], path, extra), flush=True)


# FORK: /sdcard/stereopsis/LATENCY = 1: how old the audio is when a frame starts, and when that frame is on screen
# (kms.c's page-flip time: the vblank at which it starts to scan out), per mode every 5 s. The audio process stamps
# the newest block it read and the newest analysis it published (analysis.Shared.times, CLOCK_MONOTONIC like
# kms.c's flip times); the k-th frame handed to the compositor is the k-th flip it completes.
class Latency(object):
    EVERY = 5.0

    def __init__(self, disp, an, sched=None):
        self.disp, self.an, self.sched = disp, an, sched
        self.pending = collections.deque(maxlen=32)
        self.rows = []
        self.last_done = None
        self.mode = None
        self.since = time.monotonic()

    def frame(self, mode, t_snap, t_blk, t_pub, t_handed):
        now = time.monotonic()
        if mode != self.mode or now - self.since >= self.EVERY:
            if self.mode is not None and len(self.rows) >= 20:
                self.log()
            self.mode, self.rows, self.since = mode, [], now
            self.pending.clear()
        self.pending.append((self.disp.presents, t_snap, t_blk, t_pub, t_handed))
        st = self.disp.comp_stats()
        done, flip = int(st[2]), st[16] / 1000.0
        if done != self.last_done:
            for row in self.pending:
                if row[0] == done:
                    self.rows.append(row + (flip,))
                    break
            while self.pending and self.pending[0][0] <= done:
                self.pending.popleft()
            self.last_done = done

    def log(self):
        def pct(xs, p):
            xs = sorted(xs)
            return 1000.0 * xs[min(len(xs) - 1, int(len(xs) * p))]
        r = self.rows
        blk = [x[1] - x[2] for x in r]              # the newest audio block read -> the frame starts
        ana = [x[1] - x[3] for x in r]              # the newest analysis published -> the frame starts
        draw = [x[4] - x[1] for x in r]             # the frame starts -> handed to the compositor
        show = [x[5] - x[4] for x in r]             # handed over -> on screen (flip)
        tot_b = [x[5] - x[2] for x in r]            # audio block -> on screen
        tot_a = [x[5] - x[3] for x in r]            # analysis -> on screen
        print("[stereopsis] LATENCY %s | ms p50/p90: audio block -> frame start %.1f/%.1f, analysis -> frame start "
              "%.1f/%.1f, frame start -> handed over %.1f/%.1f, handed over -> on screen %.1f/%.1f | block -> screen "
              "%.1f/%.1f (max %.1f), analysis -> screen %.1f/%.1f | %d frames" % (
                  self.mode, pct(blk, .5), pct(blk, .9), pct(ana, .5), pct(ana, .9), pct(draw, .5), pct(draw, .9),
                  pct(show, .5), pct(show, .9), pct(tot_b, .5), pct(tot_b, .9), pct(tot_b, 1.0), pct(tot_a, .5),
                  pct(tot_a, .9), len(r)) + (" | " + self.sched.state() if self.sched else " | JIT off"), flush=True)


# FORK: frame scheduling (0.8). The compositor holds one commit: a frame is committed once the frame before it is on
# screen, and shows at the vblank after its commit. Drawn as soon as the frame before it was handed over (0.2-0.7), a
# frame read its audio three refreshes before it reached the screen (LATENCY measured ~50 ms). Now, per mode, from its
# recent frames (drawing time p97, the compositor's copy):
#   just in time - a frame starts at (the vblank it is meant for) - (drawing + copy + a guard), so its audio is only
#                  that old when it goes on screen (~12-20 ms). When drawing + copy + guard fit in 90% of a refresh.
#   after commit - a frame starts as soon as the frame before is committed, and shows two vblanks later (~33 ms).
#                  When drawing + copy fit in 90% of a refresh: the frame always has a whole refresh to spare.
#   ahead        - as 0.7: a frame draws while the compositor copies the one before (~50 ms). What a heavier mode
#                  needs to hold 60 fps, and a mode at a lower rate ("@fps").
# The guard is the time a commit must come before its vblank (measured on the device: see the margin in the LATENCY
# line) plus room for a slower frame, per mode; a frame that misses its vblank by a little widens it by 1 ms (to 6 at
# most; a stall no guard could have covered does not), a minute without a miss narrows it by 0.5 ms (to 3). Five misses
# within 10 s drop the mode a tier; it tries again after 30 s (0.8.1; 0.8 stayed down until the next mode change), and
# two misses in the 10 s after that drop it again, for twice as long each time (to 8 min; a clean minute just in time
# halves the wait again). JIT = 0: always ahead, as 0.7.
class Scheduler(object):
    WINDOW = 90
    MIN_FRAMES = 45
    GUARD_MIN, GUARD_MAX = 3.0, 6.0
    CLEAN = 3600                                            # frames without a miss before the guard narrows (1 min)
    BURST, BURST_S = 5, 10.0                                # this many misses within BURST_S seconds: a tier down
    BURST_RETRY = 2                                         # ...this many, within BURST_S of trying again
    RETRY_S, RETRY_MAX_S = 30.0, 480.0                      # ...until RETRY_S later (doubling while it keeps failing)
    ORDER = ("ahead", "commit", "jit")
    NAMES = {"jit": "just in time", "commit": "after commit", "ahead": "drawing ahead"}

    def __init__(self, disp):
        self.disp = disp
        self.nominal = 1000.0 / float(disp.hz or 60)
        self.period = self.nominal
        self.periods = collections.deque(maxlen=121)
        self.mode = None
        self.draw = collections.deque(maxlen=self.WINDOW)
        self.work = collections.deque(maxlen=self.WINDOW)   # the compositor's copy + clear + OSD per frame, ms
        self.margins = collections.deque(maxlen=600)       # how long before its vblank a frame's commit came, ms
        self.tier = "ahead"
        self.cap = "jit"                                    # the best tier this mode may use (drops on misses)
        self.guard = self.GUARD_MIN                         # this mode's; the others' in guards
        self.guards = {}
        self.misses = self.clean = self.logged = 0
        self.miss_at = collections.deque()                  # when this mode's recent misses were (s, time.monotonic)
        self.retry_at = None                                # when a tier dropped for misses is tried again (s)
        self.retried_at = None                              # when it was last tried again
        self.backoff = self.RETRY_S
        # (present number, its vblank, drawn ms, late ms, tier, the drawing time planned for)
        self.targets = collections.deque(maxlen=8)
        self.target = self.last_target = None
        self.last_commit = 0.0
        self.planned = self.plan_draw = 0.0
        self.t_top = 0.0
        self.prev = None                                    # the compositor's counters at the last look

    @staticmethod
    def _pct(xs, p):
        s = sorted(xs)
        return s[min(len(s) - 1, int(len(s) * p))] if s else 0.0

    def _copy(self):
        return sum(self.work) / len(self.work) if self.work else 2.0

    def wait(self, mode, interval):
        """at the top of a frame, before anything is read: wait until this frame should start"""
        if mode != self.mode:
            if self.mode is not None:
                self.guards[self.mode] = self.guard         # each mode keeps what it learned (its own misses)
            self.mode, self.tier, self.cap, self.clean, self.logged = mode, "ahead", "jit", 0, 0
            self.miss_at.clear()
            self.retry_at, self.retried_at, self.backoff = None, None, self.RETRY_S
            self.guard = self.guards.get(mode, self.GUARD_MIN)
            self.draw.clear()
        self.target = None
        if self.tier != "ahead" and interval == 1:
            self.disp.wait_idle()                           # the frame before is committed
            st = self.disp.comp_stats()
            issued, done, flip, commit = st[1], st[2], st[16], st[17]
            if self.last_target is not None and commit > self.last_commit:
                self.margins.append(self.last_target - commit)
            self.last_commit = commit
            now = time.monotonic() * 1000.0
            d, c = self._pct(self.draw, 0.97), self._copy()
            if issued > done:        # it was just committed: it shows at the first vblank from now; this one after it
                target = flip + self.period * (max(1, math.ceil((now - flip) / self.period)) + 1)
            else:                    # nothing waiting: the first vblank this frame can make
                target = flip + self.period * math.ceil((now + d + c - flip) / self.period)
            start = now
            if self.tier == "jit":
                start = target - (d + c + self.guard)
                if start > now:
                    time.sleep(min(start - now, 2.0 * self.period) / 1000.0)
            self.target, self.planned, self.plan_draw = target, start, d
        self.last_target = self.target
        self.t_top = time.monotonic() * 1000.0

    def done(self, t_pre_ms, mode):
        """after present() returned (t_pre_ms: when it was called; mode: the mode that drew): the drawing time, the
        compositor's work, the refresh period, whether frames reach their vblank, and which tier this mode gets. A
        frame in which the mode changed (scheduled for one, drawn by the next) tells nothing about either mode"""
        drew = t_pre_ms - self.t_top
        if mode == self.mode:
            self.draw.append(drew)
            if self.target is not None:
                self.targets.append((self.disp.presents, self.target, drew, self.t_top - self.planned, self.tier,
                                     self.plan_draw))
        now_s = time.monotonic()
        st = self.disp.comp_stats()
        frames, done, work, flip = int(st[0]), int(st[2]), st[4] + st[5] + st[6], st[16]
        if self.prev is not None:
            pf, pd, pw, pflip = self.prev
            if frames > pf:
                self.work.append((work - pw) / (frames - pf))
            if done == pd + 1 and pflip > 0:                  # two flips a refresh apart: the refresh period
                gap = flip - pflip
                if 0.9 * self.nominal < gap < 1.1 * self.nominal:
                    self.periods.append(gap)
                    if len(self.periods) >= 30:
                        self.period = sorted(self.periods)[len(self.periods) // 2]
            if done > pd:
                for n, target, fdrew, late, ftier, fplan in list(self.targets):
                    if n != done:
                        continue                              # this flip is frame n's
                    if flip > target + 0.5 * self.period:
                        self._missed(now_s, flip - target, fdrew, late, ftier, fplan)
                    else:
                        self.clean += 1
                        if self.clean >= self.CLEAN:          # a minute without a miss
                            self.clean = 0
                            self.guard = max(self.GUARD_MIN, self.guard - 0.5)
                            if self.tier == "jit":             # (a clean minute lower down says nothing about it)
                                self.backoff = max(self.RETRY_S, self.backoff / 2.0)
                while self.targets and self.targets[0][0] <= done:
                    self.targets.popleft()
        self.prev = (frames, done, work, flip)
        if self.retry_at is not None and now_s >= self.retry_at:
            self.cap, self.retry_at, self.retried_at = "jit", None, now_s   # dropped for misses a while ago: try again
            print("[stereopsis] frame scheduling, %s: trying every tier again" % (self.mode,), flush=True)
        if len(self.draw) >= self.MIN_FRAMES and self.work:
            d, c, p = self._pct(self.draw, 0.97), self._copy(), self.period
            want = "jit" if d + c + self.guard < 0.9 * p else ("commit" if d + c < 0.9 * p else "ahead")
            keep = {"jit": d + c + self.guard < p, "commit": d + c < 0.97 * p, "ahead": True}
            order = self.ORDER
            want = order[min(order.index(want), order.index(self.cap))]
            if order.index(want) > order.index(self.tier) or not keep[self.tier] or \
                    order.index(self.tier) > order.index(self.cap):
                self.tier = want

    def _missed(self, now_s, by, fdrew, late, ftier, fplan):
        """a frame reached the screen `by` ms after the vblank it was planned for"""
        self.misses += 1
        self.clean = 0
        if self.logged < 20:
            self.logged += 1
            print("[stereopsis] frame scheduling, %s (%s): a frame reached the screen %.1f ms after its vblank (drawn in "
                  "%.1f ms, started %.1f ms late, guard %.1f ms)" % (self.mode, ftier, by, fdrew, late, self.guard),
                  flush=True)
        # the engine ran past its plan by `over` (started late + drawn slower than planned). Within the guard's reach it
        # is what the guard is for (or the compositor's side was slow: over <= 0): wider. A longer stall (a GC pass, a
        # busy CPU) no guard would have covered: the guard stays - such misses count only towards a burst below
        over = late + fdrew - fplan
        if ftier == "jit" and over <= self.GUARD_MAX:
            self.guard = min(self.GUARD_MAX, self.guard + 1.0)
        self.miss_at.append(now_s)
        while self.miss_at and now_s - self.miss_at[0] > self.BURST_S:
            self.miss_at.popleft()
        need = self.BURST_RETRY if self.retried_at is not None and now_s - self.retried_at < self.BURST_S else self.BURST
        if len(self.miss_at) >= need:                    # keeps missing: a tier down (below the tier that missed)
            below = self.ORDER[max(0, self.ORDER.index(ftier) - 1)]
            self.cap = self.ORDER[min(self.ORDER.index(below), self.ORDER.index(self.cap))]
            print("[stereopsis] frame scheduling, %s: %d frames late within %.0f s - at most %s, for %.0f s" % (
                self.mode, len(self.miss_at), self.BURST_S, self.NAMES[self.cap], self.backoff), flush=True)
            self.miss_at.clear()
            self.retry_at = now_s + self.backoff
            self.backoff = min(self.RETRY_MAX_S, 2.0 * self.backoff)

    def state(self):
        m = sorted(self.margins)
        return "%s, guard %.1f ms, draw p97 %.1f + compositor %.1f ms, refresh %.3f ms, misses %d%s%s" % (
            self.NAMES[self.tier], self.guard, self._pct(self.draw, 0.97), self._copy(), self.period, self.misses,
            (", at most %s for %.0f s more" % (self.NAMES[self.cap], self.retry_at - time.monotonic()))
            if self.retry_at is not None else "",
            (", commit before vblank min %.1f p5 %.1f p50 %.1f ms" % (m[0], m[len(m) // 20], m[len(m) // 2]))
            if len(m) >= 20 else "")

    def forget(self):
        """the mode on screen changed how it draws (its render size): measure it again from the start, as after a
        mode change (it keeps its guard)"""
        if self.mode is not None:
            self.guards[self.mode] = self.guard
        self.mode = None

    def brief(self):
        """for the page's /state.json (read on the preview's thread: plain values only)"""
        r = self.retry_at
        return {"tier": self.tier, "guard": round(self.guard, 1), "misses": self.misses,
                "retry_in": round(max(0.0, r - time.monotonic()), 1) if r is not None else None}


# FORK: switching engines. Shift + Persist held for a second (or the page's button) switches to the stock engine:
# where the EYESY is goes into /sdcard/stereopsis/RETURN (mode, Persist, palettes, OSD, gain), the Engine Lab's
# boot switch goes to 0 (the EYESY keeps starting with the engine chosen last), and stereopsis exits with code 1 -
# systemd restarts the service, which is the stock engine, started as at power-on. The Engine Lab, running inside
# the stock engine, opens that mode, and switches back the same way (it hands its process over, as at boot).
# /sdcard/stereopsis/SWAP = 0 turns switching off.
SWAP_HOLD_S = 1.0
LAB_NAME = "Z - Engine Lab"


def swap_enabled(ey):
    """whether switching to the stock engine can work here: allowed (SWAP), and the Engine Lab is installed
    (the stock engine needs it to come back)"""
    return (fork_flag("SWAP", "1") != "0"
            and os.path.isfile(os.path.join(ey.MODES_PATH, LAB_NAME, "main.py")))


def read_return(ey):
    """the settings a switch of engines left in RETURN, checked (an entry that does not fit is dropped); the file
    is removed. None when there is none."""
    path = os.path.join(FORK_DIR, "RETURN")
    try:
        with open(path) as f:
            r = json.load(f)
    except FileNotFoundError:
        return None
    except Exception as e:
        print("[stereopsis] RETURN unreadable (%r): ignored" % (e,), flush=True)
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
    if isinstance(g, (int, float)) and not isinstance(g, bool) and math.isfinite(g) and 0.0 <= g <= 1.0:
        out["gain"] = float(g)
    return out


def apply_return(ey, r):
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
        ey.config["audio_gain"] = r["gain"]         # as it was, not saved: config.json keeps what was saved
        ey.gain_value_snapshot = r["gain"]          # (a Shift let go of after the switch saves only a real change)
    print("[stereopsis] switched here from the stock engine: %s" % (r,), flush=True)


def write_atomic(path, text):
    tmp = path + ".tmp"
    with open(tmp, "w") as f:
        f.write(text)
        f.flush()
        os.fsync(f.fileno())
    os.replace(tmp, path)


def clear_own_markers(ey):
    """a switch is not a crash: take back the crash markers this process armed (the modes' .native_trial files
    holding our PID, stereopsis's .trial), or the next start would count them against the modes"""
    import glob
    me = str(os.getpid())
    for p in glob.glob(os.path.join(glob.escape(ey.MODES_PATH), "*", ".native_trial")) + [os.path.join(FORK_DIR, ".trial")]:
        try:
            if open(p).read().strip() == me:
                os.remove(p)
        except Exception:
            pass


def swap_to_stock(why):
    """switch to the stock engine (see above). Returns only if switching is off or cannot work here."""
    if not swap_enabled(eyesy):
        print("[stereopsis] not switching to the stock engine (%s): %s" % (
            why, "SWAP = 0" if fork_flag("SWAP", "1") == "0" else "the Engine Lab is not installed"), flush=True)
        return
    print("[stereopsis] switching to the stock engine (%s) on %s" % (why, eyesy.mode), flush=True)
    try:
        write_atomic(os.path.join(FORK_DIR, "RETURN"), json.dumps({
            "from": "stereopsis", "mode": str(eyesy.mode), "persist": not bool(eyesy.auto_clear),
            "fg": int(eyesy.fg_palette), "bg": int(eyesy.bg_palette), "osd": bool(eyesy.show_osd),
            "gain": float(eyesy.config.get("audio_gain", 0.25))}) + "\n")
        write_atomic(os.path.join(eyesy.MODES_PATH, LAB_NAME, "STEREOPSIS"), "0\n")
    except Exception as e:
        print("[stereopsis] could not write the switch files (%r): staying" % (e,), flush=True)
        try:
            os.remove(os.path.join(FORK_DIR, "RETURN"))
        except Exception:
            pass
        return
    clear_own_markers(eyesy)
    try:
        osd.loading_banner(hwscreen, "Switching to the stock engine")
    except Exception:
        pass
    try:
        if audio_process.is_alive():
            audio_process.terminate()
            audio_process.join(2)
    except Exception:
        pass
    try:
        disp.close()                                 # last: a viewer's stream only waits for frames while it runs
    except Exception as e:
        print("[stereopsis] display close: %r" % (e,), flush=True)
    print("[stereopsis] exiting with code 1: systemd starts the stock engine", flush=True)
    try:
        sys.stdout.flush()
        sys.stderr.flush()
    except Exception:
        pass
    os._exit(1)


print("starting... (stereopsis %s)" % STEREOPSIS_VERSION)

# create eyesy object
# this holds all the data (mode and preset names, knob values, midi input, sound input, current states, eyesy...)
# it gets passed to the modes which use the audio midi and knob values
eyesy = eyesy.Eyesy()
eyesy.VERSION = "stereopsis " + STEREOPSIS_VERSION   # FORK: shows on the OSD (osd.py right-aligns it)

# begin init
try :

    # see if there is a USB drive and we can run from there
    if usbdrive.mount_usb():
        print("found USB drive, checking for modes")
        if os.path.exists("/usbdrive/Modes"):
            print("found USB drive with modes, using USB")
            eyesy.GRABS_PATH =  "/usbdrive/Grabs/"
            eyesy.MODES_PATH =  "/usbdrive/Modes/"
            eyesy.SCENES_PATH = "/usbdrive/Scenes/"
            eyesy.SYSTEM_PATH = "/usbdrive/System/"
            eyesy.running_from_usb = True
        else:
            print("no modes found on USB drive, using internal")
    else:
        print("no USB found, using internal")

    eyesy.ensure_directories()

    # load config
    eyesy.load_config_file()

    # load palettes
    eyesy.load_palettes()

    # setup osc and callbacks
    osc.init(eyesy)

    # midi
    print("init midi")
    midi.init()
    eyesy.usb_midi_device = midi.input_port_usb
    print(eyesy.usb_midi_device)

    # setup alsa sound shared resources
    print("init audio")
    BUFFER_SIZE = 100
    shared_buffer = Array(c_float, BUFFER_SIZE, lock=True)  # Circular buffer, size + 1, last entry for trigger value
    shared_buffer_r = Array(c_float, BUFFER_SIZE, lock=True)  # Circular buffer, size + 1, last entry for trigger value
    write_index = Value('i', 0)     # Write index for the buffer
    gain = Value('f', 0)
    peak = Value('f', 0)
    peak_r = Value('f', 0)
    lock = Lock()
    an_shared = analysis.Shared()               # FORK: made before the fork, so both processes share it
    an_shared.attach(eyesy)                     # FORK: the attributes modes read, at rest until it runs
    audio_trig = fork_flag("TRIGGER", "peak")   # FORK: what fires eyesy.trig from audio: peak (stock), beat, both
    lat_on = fork_flag("LATENCY") == "1"        # FORK: timing logs (the Latency class; the audio process's reads)
    an_shared.stat.value = 1 if lat_on else 0

    an_run = an_shared if fork_flag("ANALYSIS", "1") != "0" else None   # FORK: ANALYSIS = 0 turns it off

    # Start the audio processing in a separate process
    audio_process = Process(target=sound.audio_processing, args=(shared_buffer, shared_buffer_r, write_index, gain, peak, peak_r, lock, an_run))
    audio_process.start()

    # init pygame, this has to happen after sound is setup
    # but before the graphics stuff below
    pygame.init()
    pygame.mouse.set_visible(False)
    clocker = pygame.time.Clock() # for locking fps

    print("pygame version " + pygame.version.ver)

    # set led to running
    osc.send("/led", 7)

    # FORK: the screen is ours. Set the mode first, then give pygame a (dummy) display surface of the
    # size we actually got, so convert() and every mode keep working exactly as before.
    disp = display.Display()
    disp.open(eyesy.RES[0], eyesy.RES[1], 60)
    if (disp.w, disp.h) != tuple(eyesy.RES):
        print("[stereopsis] the display has no %dx%d mode; using %dx%d" % (eyesy.RES[0], eyesy.RES[1], disp.w, disp.h))
        eyesy.RES = (disp.w, disp.h)
    print("opening frame buffer...")
    hwscreen = pygame.display.set_mode(eyesy.RES)
    eyesy.xres = hwscreen.get_width()
    eyesy.yres = hwscreen.get_height()
    FULL = (eyesy.xres, eyesy.yres)
    print("opened screen at: " + str(hwscreen.get_size()))
    # FORK: every pygame.display.flip() (the loading banners, the stock code paths) now shows hwscreen,
    # synchronously: the next banner draws into hwscreen straight away
    def _flip_hwscreen(*a, **k):
        rc = disp.present(hwscreen)
        disp.sync()
        return rc
    pygame.display.flip = _flip_hwscreen
    hwscreen.fill((0,0,0))
    pygame.display.flip()

    # FORK: frame surfaces, two per render size (the compositor clears one while the mode draws the other)
    scale_default, scale_modes, mode_rates = load_scale_config(FULL) if disp.comp else (FULL, {}, {})
    surfaces = {}
    cleared_with = {}                    # id(surface) -> the XRGB colour the compositor cleared it to, or None

    def surfaces_for(size):
        pair = surfaces.get(size)
        if pair is None:
            pair = surfaces[size] = [pygame.Surface(size), pygame.Surface(size)]
        return pair

    def render_size(mode_name):
        return scale_modes.get(mode_name, scale_default) if disp.comp else FULL

    osd_layers = ([pygame.Surface((display.OSD_W, display.OSD_H), pygame.SRCALPHA, 32) for _ in range(2)]
                  if disp.osd_plane else [])
    if osd_layers and osd_layers[0].get_masks() != (0xFF0000, 0xFF00, 0xFF, 0xFF000000):
        print("[stereopsis] OSD surface is not ARGB8888 (%s): OSD drawn into the frame instead" % (osd_layers[0].get_masks(),))
        osd_layers = []
    if scale_modes or scale_default != FULL:
        print("[stereopsis] render sizes: default %s, %s" % (scale_default, scale_modes), flush=True)
    if mode_rates:
        print("[stereopsis] frame rates: %s" % (mode_rates,), flush=True)

    # screen for mode to draw on
    mode_screen = surfaces_for(FULL)[0]

    # eyesy gets a refrence to screen so it can save screen grabs
    eyesy.screen = mode_screen#hwscreen
    print(str(eyesy.screen) + " " +  str(hwscreen))

    # load modes, post banner if none found
    if not (eyesy.load_modes()) :
        print("no modes found.")
        osd.loading_banner(hwscreen, "No Modes found. Insert USB drive with Modes folder and restart.")
        while True:
            # quit on esc
            for event in pygame.event.get():
                if event.type == QUIT:
                    exitexit(0)
                elif event.type == KEYDOWN:
                    if event.key == K_ESCAPE:
                        exitexit(0)
            time.sleep(1)

    # run setup functions if modes have them
    setup_size = {}                                  # FORK: the size each mode's setup() last ran at
    # FORK: /sdcard/stereopsis/LAZY = 1: no setups here - each mode is set up when it is first shown (the
    # main loop sets up any mode setup_size has no entry for). Boots ~7 s sooner with 111 modes and holds
    # only the shown modes' images; the first visit to a mode waits for its setup
    lazy = fork_flag("LAZY") == "1"
    print("running setup..." if not lazy else "[stereopsis] LAZY: modes are set up when first shown", flush=True)
    for i in range(0, len(eyesy.mode_names) if not lazy else 0) :
        print(eyesy.mode_root)
        try :
            eyesy.set_mode_by_index(i)
            mode = sys.modules[eyesy.mode]
        except AttributeError :
            print("mode not found, or has error")
            continue
        try :
            osd.loading_banner(hwscreen,"Loading " + str(eyesy.mode) )
            print("setup " + str(eyesy.mode))
            eyesy.xres, eyesy.yres = render_size(eyesy.mode)       # FORK: set up at the size it will draw at
            setup_size[eyesy.mode] = (eyesy.xres, eyesy.yres)
            if (eyesy.xres, eyesy.yres) != FULL:
                print("[stereopsis] %s renders at %dx%d" % (eyesy.mode, eyesy.xres, eyesy.yres), flush=True)
            mode.setup(hwscreen, eyesy)
            eyesy.memory_used = psutil.virtual_memory()[2]
        except Exception as e:
            print("error in setup, or setup not found")
            print(traceback.format_exc())
            continue
    eyesy.xres, eyesy.yres = FULL

    # load screen grabs
    eyesy.load_grabs()

    # load scenes
    eyesy.load_scenes()

    # set font for system stuff
    eyesy.font = pygame.font.Font("font.ttf", 16)

    # get total memory consumed, cap at 75%
    eyesy.memory_used = psutil.virtual_memory()[2]
    eyesy.memory_used = (eyesy.memory_used / 75) * 100
    if (eyesy.memory_used > 100): eyesy.memory_used = 100

    # set initial mode
    eyesy.set_mode_by_index(0)
    # FORK: or the mode (and Persist, palettes, OSD, gain) a switch from the stock engine left in RETURN
    try:
        ret = read_return(eyesy)
        if ret:
            apply_return(eyesy, ret)
    except Exception as e:
        print("[stereopsis] RETURN not applied: %r" % (e,), flush=True)
    mode = sys.modules[eyesy.mode]

    # for flashing the LED
    midi_led_flashing = False

    # LFO for simulated sound
    undulate_p = 0

    # menu screens, need to load after pygame
    eyesy.menu_screens["home"] = ScreenMainMenu(eyesy)
    eyesy.menu_screens["test"] = ScreenTest(eyesy)
    eyesy.menu_screens["video_settings"] = ScreenVideoSettings(eyesy)
    eyesy.menu_screens["palette"] = ScreenPalette(eyesy)
    eyesy.menu_screens["wifi"] = ScreenWiFi(eyesy)
    eyesy.menu_screens["applogs"] = ScreenApplogs(eyesy)
    eyesy.menu_screens["midi_settings"] = ScreenMIDISettings(eyesy)
    eyesy.menu_screens["midi_pc_mapping"] = ScreenMIDIPCMapping(eyesy)
    eyesy.menu_screens["flashdrive"] = ScreenFlashDrive(eyesy)
    eyesy.switch_menu_screen("home")

    # FORK: the browser preview, and its controls (applied by the main loop)
    remote = page = None
    if disp.preview and fork_flag("PREVIEW", "1") != "0":
        import preview
        try:
            pv = preview.Preview(disp, eyesy, int(fork_flag("PREVIEW_PORT", "8081") or 8081),
                                 controls=fork_flag("CONTROLS", "1") != "0", swap_ok=lambda: swap_enabled(eyesy))
            if pv.start():
                page = pv
                if pv.controls:
                    remote = pv
        except Exception as e:
            print("[stereopsis] browser preview did not start: %r" % (e,), flush=True)

    # used to measure fps
    start = time.time()
    frame_t = None                  # FORK: when the previous frame began (eyesy.dt / step / tick)
    tick_acc = 0.0
    rand_state = rand_end = None    # FORK: the random generator at the start and end of the last tick frame

    # FORK: frame cap, meter, crash marker, the frame pipeline's state
    try:
        fps_cap = int(float(fork_flag("FPSCAP", "60")))
    except Exception:
        fps_cap = 60
    fps_cap = max(10, min(120, fps_cap))
    meter = Meter(disp) if fork_flag("METER") == "1" else None
    sched = Scheduler(disp) if disp.comp and not disp.null and fork_flag("JIT", "1") != "0" else None
    if page is not None:
        page.sched = sched                           # /state.json shows the current mode's tier
    latency = Latency(disp, an_shared, sched) if lat_on and disp.comp and not disp.null else None
    good_frames = 0
    present_errors = 0
    turn = 0                    # which of the two surfaces this frame draws into
    last_frame = None           # the surface presented last (the compositor may still be reading it)
    last_osd = None
    was_persist = False
    print("[stereopsis] running: frame cap %d, meter %s, compositor %s, OSD layer %s" % (
        fps_cap, "on" if meter else "off", "on" if disp.comp else "off", "on" if osd_layers else "off"), flush=True)

except Exception as e:
    print(traceback.format_exc())
    print("error with EYESY init")
    exitexit(1)                                  # FORK: non-zero, so systemd restarts (the stock engine)

while 1:

    # FORK: just-in-time frames (Scheduler): wait until this frame should start
    if sched is not None:
        try:
            _rate = mode_rates.get(eyesy.mode, fps_cap)
            sched.wait(eyesy.mode, max(1, min(4, int(round((disp.hz or 60) / float(_rate))))))
        except Exception as e:
            print("[stereopsis] frame scheduling off: %r" % (e,), flush=True)
            sched = None
            if page is not None:
                page.sched = None

    # quit on esc
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            exitexit(0)
        elif event.type == pygame.KEYDOWN:
            if event.key == pygame.K_ESCAPE:
                exitexit(0)
    # main loop
    try :
        # check for OSC
        # key events will be dispatched from here
        # knobs from hardware
        osc.recv()

        # check MIDI
        # matching CC merged with knobs
        midi.recv_ttymidi(eyesy)
        midi.recv_usbmidi(eyesy)

        # FORK: what the browser preview's controls asked for since the last frame
        if remote is not None:
            remote.apply()
            if remote.render_request:                # the page's Size buttons (0.9)
                preset, remote.render_request = remote.render_request, None
                if disp.comp:
                    set_render_size(eyesy.mode, preset)

        # FORK: switching engines: Shift + Persist held for a second, or the page's button (swap_to_stock does not
        # return unless switching is off here)
        if eyesy.swap_hold_t0 is not None and not eyesy.menu_mode and time.monotonic() - eyesy.swap_hold_t0 >= SWAP_HOLD_S:
            eyesy.swap_hold_t0 = None
            eyesy.swap_request = "Shift + Persist"
        if eyesy.swap_request:
            why, eyesy.swap_request = eyesy.swap_request, None
            swap_to_stock(why)

        # get knobs, checking for override, and check for new note on
        # for the knobs, only changes are assinged
        eyesy.update_knobs_and_notes()

        # for repeating keys held down
        eyesy.update_key_repeater()

        # check gain knob
        eyesy.check_gain_knob()

        # sequence the knobs
        # only changes assigned
        eyesy.knob_seq_run()

        # fills in eyesy.knob1 etc, for the modes
        eyesy.set_knobs()

        # measure fps
        eyesy.frame_count += 1
        if ((eyesy.frame_count % 30) == 0):
            now = time.time()
            eyesy.fps = 1 / ((now - start) / 30)
            start = now

        # FORK: this frame's time for the modes, in whole display refreshes since the previous frame (at least
        # one: a frame cannot come sooner; at most 0.1 s, so a stall - a setup, a mode switch - is no jump)
        now_t = time.perf_counter()
        hz = disp.hz or 60
        refreshes = 1 if frame_t is None else min(max(1, int(round((now_t - frame_t) * hz))), max(1, int(hz // 10)))
        frame_t = now_t
        eyesy.dt = refreshes / float(hz)
        eyesy.step = eyesy.dt * 30.0
        tick_acc += eyesy.step
        eyesy.tick = tick_acc >= 1.0 - 1e-6
        if eyesy.tick:
            tick_acc = min(tick_acc - 1.0, 0.999)
        eyesy.lfo_frame(eyesy.tick)                  # the colour LFO at stock's rate (eyesy.py)

        # update new led
        if (eyesy.new_led) :
            osc.send("/led", eyesy.led)

        # FORK: the newest audio analysis onto eyesy (levels, bands, beat, tempo), before anything uses it
        an_shared.update(eyesy, lock)
        t_snap, t_blk, t_pub = time.monotonic(), an_shared.times[1], an_shared.times[0]   # (LATENCY)

        # get sound and trigger, unless trigger button is being pressed, then do the simulated sound
        trig_before = bool(eyesy.trig)                 # FORK: a trigger already here came from a key, MIDI or the page
        if not eyesy.key10_status:
            with lock:
                eyesy.audio_in[:] = shared_buffer[:]
                eyesy.audio_in_r[:] = shared_buffer_r[:]
                g = eyesy.config["audio_gain"]
                gain.value = float((g * g * 50) + 1)  # map audio, make it big
                # update audio trig and peak
                eyesy.audio_peak = peak.value
                eyesy.audio_peak_r = peak_r.value
                # trigger source 0 = audio, 2 = audio or notes
                if (eyesy.config["trigger_source"] == 0 or eyesy.config["trigger_source"] == 2):
                    # FORK: /sdcard/stereopsis/TRIGGER = peak (stock: loud), beat (a detected kick), both. The
                    # loudness trigger fires at most once per stock frame (eyesy.tick): above 30 fps a loud
                    # passage would fire more triggers than on stock, and trigger modes draw on every one (the
                    # peak holds for 50 ms, so no hit falls between two ticks)
                    if audio_trig != "beat" and eyesy.tick and (eyesy.audio_peak > 20000 or eyesy.audio_peak_r > 20000) : eyesy.trig = True
                    if audio_trig != "peak" and eyesy.beat : eyesy.trig = True
        else:
            # dont do simulated sound in menu mode cause it interferes with test screen
            if not eyesy.menu_mode :
                undulate_p += .005
                undulate = ((math.sin(undulate_p * 2 * math.pi) + 1) * 2) + .5
                for i,v in enumerate(eyesy.audio_in):
                    eyesy.audio_in[i] = int(math.sin((i / 100) * 2 * math.pi * undulate) * 25000)
                    eyesy.audio_in_r[i] = eyesy.audio_in[i]
                eyesy.audio_peak = 25000 # also set peak value
                eyesy.audio_peak_r = 25000 # also set peak value
        # FORK (0.9.1): eyesy.trig_audio = this frame's trigger came only from the audio (the loudness trigger, or a
        # detected kick with TRIGGER = beat/both), not from the Trigger button, MIDI or the page. A mode whose Trigger
        # is a performer's gesture acts on `eyesy.trig and not eyesy.trig_audio` (on a hot input the loudness trigger
        # fires nearly every frame)
        eyesy.trig_audio = bool(eyesy.trig) and not trig_before

        # set the mode on which to call draw
        try :
            mode = sys.modules[eyesy.mode]
        except :
            eyesy.error = "Mode " + eyesy.mode  + " not loaded, probably has errors."
            print(eyesy.error)
            # no use spitting these errors out at 30 fps
            pygame.time.wait(200)

        # FORK: pick this frame's surface: the mode's render size, the one the compositor is not using
        size = render_size(eyesy.mode)
        if (eyesy.xres, eyesy.yres) != size:
            eyesy.xres, eyesy.yres = size
        if not eyesy.run_setup and setup_size.get(eyesy.mode) != size:
            eyesy.run_setup = True                   # set it up (again) for the size it draws at; LAZY: first shown
            print("[stereopsis] %s: setup %s at %dx%d" % (eyesy.mode, "again" if eyesy.mode in setup_size else "on first show",
                                                          size[0], size[1]), flush=True)
        if eyesy.run_setup:
            setup_size[eyesy.mode] = size
        pair = surfaces_for(size)
        persist = not eyesy.auto_clear
        mode_screen = pair[0] if persist else pair[turn]
        if mode_screen is last_frame or last_frame is hwscreen:
            disp.sync()                              # never draw into what the compositor still reads
                                                     # (setup(), the menu and the OSD fallback draw into hwscreen)
        if persist and not was_persist and last_frame is not None and last_frame.get_size() == size \
                and cleared_with.get(id(last_frame)) is not None:
            disp.snapshot(mode_screen)               # persist continues from the last frame, as in stock
            cleared_with[id(mode_screen)] = None
        was_persist = persist

        # save a screen shot before drawing stuff
        if (eyesy.screengrab_flag):
            if disp.comp and not disp.null and last_frame is not None:     # FORK: grab what is on screen
                grab = pygame.Surface(last_frame.get_size())
                disp.snapshot(grab)
                eyesy.screen = grab
            eyesy.screengrab()

        # see if save is being held down for deleting scene
        eyesy.update_scene_save_key()

        # clear it with bg color if auto clear enabled
        if eyesy.auto_clear :
            #hwscreen.fill(eyesy.bg_color)
            # FORK: skipped when the compositor already cleared this surface to exactly this colour
            if cleared_with.get(id(mode_screen)) != display.xrgb(eyesy.bg_color):
                mode_screen.fill(eyesy.bg_color)

        # run setup (usually if the mode was reloaded)
        if eyesy.run_setup :
            eyesy.error = ''
            try :
                mode.setup(hwscreen, eyesy)
            except Exception as e:
                eyesy.error = traceback.format_exc()
                print("error with setup: " + eyesy.error)

        # FORK: which surface goes on screen, whether the compositor clears it afterwards, the OSD layer
        frame, clear, osd_layer = mode_screen, None, None

        # draw it
        if not eyesy.menu_mode :
            try :
                # FORK: randomness at stock's rate. A frame that does not complete a stock frame (eyesy.tick
                # False: every other frame at 60 fps) replays the random numbers of the tick frame before it,
                # then leaves the generator where that tick frame did, as if it had drawn nothing: whatever a
                # mode draws at random - and eyesy.color_picker's random colours - changes 30 times a second,
                # as on stock (at 60 it flickered twice as fast)
                replay = not eyesy.tick and rand_state is not None
                if eyesy.tick:
                    rand_state = random.getstate()
                else:
                    if replay:
                        random.setstate(rand_state)
                try:
                    #mode.draw(hwscreen, eyesy)
                    mode.draw(mode_screen, eyesy)
                finally:
                    if eyesy.tick:
                        rand_end = random.getstate()
                    elif replay and rand_end is not None:
                        random.setstate(rand_end)
            except Exception as e:
                eyesy.error = traceback.format_exc()
                print("error with draw: " + eyesy.error)
                # no use spitting these errors out at 30 fps
                pygame.time.wait(200)

            if eyesy.auto_clear and disp.comp:
                clear = eyesy.bg_color                # the colour stock would fill the next frame with
            if eyesy.show_osd and not osd_layers:    # FORK: no OSD layer: stock's way, onto the display surface
                if size == FULL:
                    hwscreen.blit(mode_screen, (0,0))
                else:
                    pygame.transform.scale(mode_screen, FULL, hwscreen)
                frame, clear = hwscreen, None

        # osd
        if eyesy.show_osd and not eyesy.menu_mode:
            try :
                if osd_layers:                       # FORK: on its own transparent layer, not in the frame
                    osd_layer = osd_layers[turn]
                    if osd_layer is last_osd:
                        disp.sync()
                    osd_layer.fill((0, 0, 0, 0))
                    osd.render_overlay_480(osd_layer, eyesy)
                else:
                    osd.render_overlay_480(hwscreen, eyesy)
            except Exception as e:
                eyesy.error = traceback.format_exc()
                print("error with OSD: " + eyesy.error)
                pygame.time.wait(200)

        if eyesy.menu_mode :
            frame, clear, osd_layer = hwscreen, None, None
            try:
                eyesy.current_screen.handle_events()
                eyesy.current_screen.render_with_title(hwscreen)
            except Exception as e:
                eyesy.error = traceback.format_exc()
                print("error with Menu: " + eyesy.error)
                pygame.time.wait(200)
            # menu might signal restart
            if eyesy.restart :
                print("restart requested from menu, restarting")
                exitexit(1)
            # menu exits, clear screen
            if not eyesy.menu_mode :
                hwscreen.fill(eyesy.bg_color)

        # FORK: hand the frame to the compositor (it copies, clears and commits on its own core). A mode's frame
        # rate below the display's (FPSCAP, or "@fps" in scale.json) is a swap interval: the compositor shows the
        # frame that many refreshes after the previous one, locked to the display
        rate = mode_rates.get(eyesy.mode, fps_cap)
        interval = max(1, min(4, int(round((disp.hz or 60) / float(rate)))))
        t_present = time.perf_counter()
        t_pre = time.monotonic() * 1000.0              # (the Scheduler: this frame's drawing ends here)
        rc = disp.present(frame, clear=clear, osd=osd_layer, interval=interval)
        t_done = time.perf_counter()
        cleared_with[id(frame)] = display.xrgb(clear) if clear is not None else None
        last_frame, last_osd = frame, osd_layer
        if disp.comp:
            turn ^= 1                                # legacy presents are synchronous: one surface is enough
        if rc != 0:
            present_errors += 1
            print("[stereopsis] present failed: %d" % rc, flush=True)
            if present_errors >= 30:
                print("[stereopsis] the display keeps failing - exiting so the stock engine takes over", flush=True)
                exitexit(1)
        elif good_frames < TRIAL_FRAMES:
            good_frames += 1
            if good_frames == TRIAL_FRAMES:
                try:
                    os.remove(os.path.join(FORK_DIR, ".trial"))
                    print("[stereopsis] %d good frames: crash marker cleared" % TRIAL_FRAMES, flush=True)
                except Exception:
                    pass
        if meter is not None:
            meter.tick(eyesy.mode, t_present, t_done, "STEREOPSIS")
        if sched is not None and rc == 0:
            try:
                sched.done(t_pre, eyesy.mode)
            except Exception as e:
                print("[stereopsis] frame scheduling off: %r" % (e,), flush=True)
                sched = None
                if page is not None:
                    page.sched = None
        if latency is not None and rc == 0:
            latency.frame(eyesy.mode, t_snap, t_blk, t_pub, time.monotonic())

        # clear all the events
        eyesy.clear_flags()

    except Exception as e:
        eyesy.clear_flags() # don't keep doing things
        eyesy.error = traceback.format_exc()
        print("problem in main loop")
        print(eyesy.error)
        pygame.time.wait(200)

    # FORK: limit fps. With the compositor, it paces the loop (a frame is handed over only once the previous
    # one is on its way, at the swap interval above) and tick(0) only keeps the clock's books; without it (the
    # legacy path, the null display of the tests) tick() sleeps like the stock 30 fps cap did.
    rate = mode_rates.get(eyesy.mode, fps_cap)
    clocker.tick(0 if (disp.comp and not disp.null) or rate >= disp.hz else rate)

time.sleep(1)

print("Quit")
