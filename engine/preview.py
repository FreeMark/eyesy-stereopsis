"""preview.py - stereopsis's browser preview and remote. Open http://<the EYESY's address>:8081/ on a phone or
computer on the same network: the live picture (MJPEG) with the mode and frame rate on top, and under it
(beside it on a wide screen) the EYESY's controls: the five knobs, mode, scene, palettes, OSD, Persist,
Trigger, screen grab and audio gain, the size the mode on screen draws at (0.9: the display's, half, a quarter; saved
per mode), the text of a mode that writes one (0.10: a mode with get_text() / set_text()) - and a button that switches
to the stock engine on the same mode (while the stock engine runs, the Engine Lab answers on this address with a small
page that switches back).

The frames come from the compositor (kms.c): downscaled and JPEG-encoded on spare CPU cores, only while
someone is watching, so an unwatched preview costs nothing. This file only serves them. It runs as a daemon
thread inside the engine; each viewer gets its own thread (the ctypes calls into kms.c release the GIL).

Controls travel the other way. The page POSTs small JSON actions to /control; this file checks them and
queues them, and the engine's main loop applies the queue once per frame (apply(), on the main thread, so
nothing here changes the engine's state from another thread). A knob set from the page is held the way a
MIDI CC holds one (stock eyesy.cc_override_knob) until that knob on the EYESY is turned, which takes it back.

  /             the page
  /stream.mjpg  multipart JPEG stream (what the page shows)
  /frame.jpg    one current frame
  /state.json   mode, fps, knobs (and which are held), render size (and the display's), OSD / persist, scene,
                palettes, gain, audio (stock peaks + the engine's analysis: levels, 32 bands, beats, tempo), the
                frame scheduler's tier, perf totals, the mode's text (null for a mode that writes none)
  /modes.json   the installed modes, in the EYESY's order
  /control      POST, JSON: one action or a list of up to 32 (see parse_action)
Like the EYESY's web editor, it has no password: keep the EYESY on a network you trust. /control only takes
JSON (so another web site cannot post a plain form to it) and refuses requests from pages on other origins.
/sdcard/stereopsis/CONTROLS = 0 makes the page view-only.
"""
import collections
import ctypes
import json
import math
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

MAX_BODY = 4096              # a /control request; larger ones are refused (413)
DRAIN_MAX = 1 << 20          # up to this much of a refused body is read and dropped before the reply
MAX_ACTIONS = 32

PAGE = r"""<!doctype html>
<html lang="en"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1,viewport-fit=cover">
<title>EYESY &middot; stereopsis</title>
<style>
:root { color-scheme: dark; --bg: #0b0b0d; --panel: #141418; --ctl: #212128; --ctl2: #2d2d36; --line: #33333d;
  --text: #e8e8ec; --dim: #9a9aa6; --accent: #8ab4f8; --held: #f2b35e; --on: #7fd1a0; }
* { box-sizing: border-box; }
html, body { margin: 0; height: 100%; background: var(--bg); color: var(--text);
  font: 14px/1.35 system-ui, -apple-system, "Segoe UI", sans-serif; }
body { display: flex; flex-direction: column; height: 100dvh; overflow: hidden; }
#screen { position: relative; flex: 0 0 auto; height: min(56.25vw, 52dvh); background: #000; }
body.bare #screen, body.view #screen { flex: 1 1 auto; height: auto; }
#v { position: absolute; inset: 0; width: 100%; height: 100%; object-fit: contain; }
#hud { position: absolute; left: 8px; top: 8px; max-width: calc(100% - 110px); padding: 6px 10px;
  border-radius: 8px; background: rgba(0, 0, 0, .62); display: flex; flex-wrap: wrap; gap: 2px 10px;
  align-items: center; transition: opacity .25s; pointer-events: none; }
#screen.nohud #hud { opacity: 0; }
#mode { font-weight: 600; }
#meta { color: var(--dim); font-variant-numeric: tabular-nums; }
#down { position: absolute; inset: 0; display: none; place-items: center; padding: 16px; text-align: center;
  color: var(--text); background: rgba(11, 11, 13, .78); }
#tog { position: absolute; right: 8px; top: 8px; min-height: 34px; padding: 0 10px; font-size: 12px;
  background: rgba(0, 0, 0, .62); }
body.view #tog { display: none; }
#panel { flex: 1 1 auto; overflow-y: auto; background: var(--panel); border-top: 1px solid var(--line);
  padding: 10px 12px calc(12px + env(safe-area-inset-bottom)); display: grid; gap: 8px; align-content: start; }
body.bare #panel, body.view #panel { display: none; }
#panel.off { opacity: .45; pointer-events: none; }
.row { display: flex; gap: 8px; align-items: center; min-width: 0; }
button, select { font: inherit; color: var(--text); background: var(--ctl); border: 1px solid var(--line);
  border-radius: 8px; min-height: 44px; padding: 0 12px; touch-action: manipulation; }
button { cursor: pointer; }
button:active { background: var(--ctl2); }
button[aria-pressed="true"] { border-color: var(--on); color: var(--on); }
button:disabled { opacity: .35; cursor: default; }
.step { min-width: 44px; padding: 0; font-size: 22px; line-height: 1; }
select { flex: 1 1 auto; min-width: 0; }
.knob { display: grid; grid-template-columns: 38px 1fr 44px; gap: 8px; align-items: center; }
.knob b { color: var(--dim); font-weight: 600; font-size: 12px; }
.knob output { text-align: right; font-variant-numeric: tabular-nums; color: var(--dim); }
.knob.held b, .knob.held output { color: var(--held); }
.track { position: relative; }
.track .hw { position: absolute; top: 50%; left: calc(8px + (100% - 16px) * var(--p, 0)); width: 2px;
  height: 20px; margin: -10px 0 0 -1px; border-radius: 1px; background: var(--held); opacity: 0;
  pointer-events: none; transition: left .2s; }
.knob.held .hw { opacity: .85; }
input[type=range] { display: block; width: 100%; height: 40px; margin: 0; background: transparent;
  accent-color: var(--accent); touch-action: pan-y; }
.knob.held input[type=range] { accent-color: var(--held); }
#trig { flex: 1 1 auto; font-weight: 700; letter-spacing: .06em; }
#trig.flash { background: var(--accent); color: #000; border-color: var(--accent); }
.lbl { color: var(--dim); font-size: 12px; width: 44px; flex: 0 0 auto; }
.val { flex: 1 1 auto; min-width: 0; overflow: hidden; text-overflow: ellipsis; white-space: nowrap;
  text-align: center; }
#release, #swap, #textset, #textreset { flex: 0 0 auto; }
#text { flex: 1 1 auto; min-width: 0; font: inherit; color: var(--text); background: var(--ctl);
  border: 1px solid var(--line); border-radius: 8px; min-height: 44px; padding: 0 10px; }
.size { flex: 1 1 0; min-width: 0; padding: 0 6px; font-variant-numeric: tabular-nums; }
#swap.armed { border-color: var(--held); color: var(--held); }
#meter { display: grid; grid-template-columns: 38px 1fr 64px; gap: 8px; align-items: center; }
#meter b { color: var(--dim); font-weight: 600; font-size: 12px; }
#spec { display: block; width: 100%; height: 34px; }
#bpm { text-align: right; font-variant-numeric: tabular-nums; color: var(--dim); font-size: 12px; transition: color .15s; }
#bpm.beat { color: var(--accent); }
.hint { color: var(--dim); font-size: 12px; }
#foot { color: #6d6d78; font-size: 11px; }
@media (min-aspect-ratio: 4/3) {
  body { flex-direction: row; }
  #screen { flex: 1 1 auto; height: auto; }
  #panel { flex: 0 0 auto; width: min(370px, 46vw); border-top: 0; border-left: 1px solid var(--line); }
}
</style></head>
<body>
<div id="screen">
  <img id="v" src="/stream.mjpg" alt="The EYESY's video output">
  <div id="hud"><span id="mode">connecting&hellip;</span><span id="meta"></span></div>
  <div id="down">waiting for the EYESY&hellip;</div>
  <button id="tog" aria-expanded="true" title="show or hide the controls">controls</button>
</div>
<div id="panel" aria-label="EYESY controls">
  <div class="row">
    <button class="step" data-a="mode" data-s="-1" aria-label="previous mode">&lsaquo;</button>
    <select id="modes" aria-label="mode"></select>
    <button class="step" data-a="mode" data-s="1" aria-label="next mode">&rsaquo;</button>
  </div>
  <div id="knobs"></div>
  <div class="row">
    <button id="trig" title="Trigger (T or space)">TRIGGER</button>
    <button id="osd" data-a="osd" aria-pressed="false" title="on-screen display (O)">OSD</button>
    <button id="persist" data-a="persist" aria-pressed="false" title="Persist: stop clearing between frames (P)">Persist</button>
    <button data-a="grab" title="save a screen grab on the EYESY (Grabs folder)">Grab</button>
  </div>
  <div class="row"><span class="lbl">Scene</span>
    <button class="step sc" data-a="scene" data-s="-1" aria-label="previous scene">&lsaquo;</button>
    <span class="val" id="scene">&ndash;</span>
    <button class="step sc" data-a="scene" data-s="1" aria-label="next scene">&rsaquo;</button></div>
  <div class="row"><span class="lbl">Colour</span>
    <button class="step" data-a="palette" data-w="fg" data-s="-1" aria-label="previous foreground palette">&lsaquo;</button>
    <span class="val" id="fg">&ndash;</span>
    <button class="step" data-a="palette" data-w="fg" data-s="1" aria-label="next foreground palette">&rsaquo;</button></div>
  <div class="row"><span class="lbl">Bg</span>
    <button class="step" data-a="palette" data-w="bg" data-s="-1" aria-label="previous background palette">&lsaquo;</button>
    <span class="val" id="bg">&ndash;</span>
    <button class="step" data-a="palette" data-w="bg" data-s="1" aria-label="next background palette">&rsaquo;</button></div>
  <div class="row" id="sizerow" hidden title="what this mode draws at - saved for this mode; smaller is faster"><span class="lbl">Size</span>
    <button class="size" data-size="full" aria-pressed="false">full</button>
    <button class="size" data-size="half" aria-pressed="false">half</button>
    <button class="size" data-size="quarter" aria-pressed="false">quarter</button></div>
  <form class="row" id="textrow" hidden title="what this mode writes - kept for this mode"><span class="lbl">Text</span>
    <input id="text" type="text" maxlength="64" autocomplete="off" spellcheck="false" enterkeyhint="done"
      aria-label="the text this mode writes">
    <button id="textset" type="submit">Set</button>
    <button id="textreset" type="button" title="back to the text in the mode's file">Reset</button></form>
  <div id="gainrow"></div>
  <div id="meter" title="the engine's audio analysis: 32 bands from 50 Hz to 16 kHz, beats, tempo"><b>Audio</b>
    <canvas id="spec" aria-label="audio spectrum"></canvas><span id="bpm">&ndash;</span></div>
  <div class="row"><button id="release" disabled>Hand the knobs back</button>
    <span class="hint">Amber knobs are held (by this page, a scene or MIDI) until that knob on the EYESY is turned.</span></div>
  <div class="row" id="swaprow" hidden><button id="swap" title="switch to Critter &amp; Guitari's original engine, on this mode">Stock engine</button>
    <span class="hint" id="swaphint">Compare with the original engine on this mode: about 15 s to switch, 10 s back.
    On the EYESY: hold Shift + Persist for a second.</span></div>
  <div id="foot">keys: &larr; &rarr; mode &middot; T trigger &middot; O OSD &middot; P persist &middot; tap the picture to hide its label</div>
</div>
<script>
"use strict";
const $ = (id) => document.getElementById(id);
const v = $("v"), screen = $("screen"), panel = $("panel"), sel = $("modes"), trig = $("trig");
const DOT = " · ", TIMES = "×", DASH = "–";
const busy = {};                 // control -> when it was last touched here (Infinity while held)
let modeCount = -1, loadingModes = false;

function post(acts) {
  return fetch("/control", { method: "POST", headers: { "Content-Type": "application/json" },
    body: JSON.stringify(acts) }).catch(() => {});
}
function act(a) { post([a]); }

// sliders: the newest value per control, one request in flight, about 25 a second at most
const pend = new Map();
let timer = 0, inflight = false;
function queue(key, a) {
  pend.set(key, a);
  if (busy[key] !== Infinity) busy[key] = performance.now();
  if (!timer) timer = setTimeout(flush, 35);
}
async function flush() {
  timer = 0;
  if (inflight) { timer = setTimeout(flush, 35); return; }
  if (!pend.size) return;
  const acts = [...pend.values()];
  pend.clear();
  inflight = true;
  await post(acts);
  inflight = false;
}
function fresh(key) { return !(key in busy) || performance.now() - busy[key] > 800; }
function release() { for (const k in busy) if (busy[k] === Infinity) busy[k] = performance.now(); }
window.addEventListener("pointerup", release);
window.addEventListener("pointercancel", release);

function slider(key, label, name, onInput, onDone) {
  const row = document.createElement("div"), b = document.createElement("b"), tr = document.createElement("div");
  const inp = document.createElement("input"), hw = document.createElement("i"), out = document.createElement("output");
  row.className = "knob"; tr.className = "track"; hw.className = "hw";
  b.textContent = label;
  inp.type = "range"; inp.min = "0"; inp.max = "1"; inp.step = "0.001"; inp.value = "0";
  inp.setAttribute("aria-label", name);
  tr.append(inp, hw); row.append(b, tr, out);
  inp.addEventListener("pointerdown", () => { busy[key] = Infinity; });
  inp.addEventListener("input", () => { out.textContent = (+inp.value).toFixed(2); onInput(+inp.value); });
  if (onDone) inp.addEventListener("change", () => onDone(+inp.value));
  return { row, inp, hw, out };
}
const knobs = [];
for (let i = 0; i < 5; i++) {
  const k = slider("k" + i, "K" + (i + 1), "knob " + (i + 1), (x) => queue("k" + i, { act: "knob", i: i, v: x }));
  knobs.push(k);
  $("knobs").append(k.row);
}
const gain = slider("g", "Gain", "audio gain", (x) => queue("g", { act: "gain", v: x }),
  (x) => queue("g", { act: "gain", v: x, save: true }));
$("gainrow").replaceWith(gain.row);

document.querySelectorAll("[data-a]").forEach((b) => b.addEventListener("click", () => {
  const a = { act: b.dataset.a };
  if (b.dataset.s) a.step = +b.dataset.s;
  if (b.dataset.w) a.which = b.dataset.w;
  if (a.act === "mode") busy.mode = performance.now();
  act(a);
}));
function fire() {
  act({ act: "trigger" });
  trig.classList.add("flash");
  setTimeout(() => trig.classList.remove("flash"), 90);
}
trig.addEventListener("pointerdown", (e) => { e.preventDefault(); fire(); });
trig.addEventListener("click", (e) => { if (e.detail === 0) fire(); });      // keyboard
sel.addEventListener("change", () => { busy.mode = performance.now(); act({ act: "mode", name: sel.value }); });
$("release").addEventListener("click", () => act({ act: "release" }));
const SIZES = { full: 1, half: 2, quarter: 4 };
document.querySelectorAll(".size").forEach((b) => b.addEventListener("click", () => act({ act: "render", size: b.dataset.size })));
// the Text row (0.10): a mode that writes a text; the box is left alone while you type in it
const textrow = $("textrow"), textin = $("text");
let curMode = "", shownText = null;
textrow.addEventListener("submit", (e) => {
  e.preventDefault();
  if (!textin.value.trim()) return;
  busy.text = performance.now();
  act({ act: "text", value: textin.value, mode: curMode });
  textin.blur();
});
$("textreset").addEventListener("click", () => { busy.text = performance.now(); act({ act: "text", reset: true, mode: curMode }); });

// switching to the stock engine: the first click arms the button for 4 s, the second switches; this page then
// waits for the stock engine's small page, which answers on the same address, and reloads into it
const swapBtn = $("swap");
let swapArmed = -1e9, swapping = false, swapAt = 0, noteUntil = 0;
function disarm() { swapBtn.classList.remove("armed"); swapBtn.textContent = "Stock engine"; }
function overlay(msg) { $("down").textContent = msg; $("down").style.display = "grid"; }
function showSwapping() { overlay("Switching to the stock engine… this page follows in about 15 s"); }
function note(msg) { swapping = false; overlay(msg); noteUntil = performance.now() + 4000; }     // shown 4 s
function quiet() { return !swapping && performance.now() > noteUntil; }
swapBtn.addEventListener("click", () => {
  if (performance.now() - swapArmed > 4000) {
    swapArmed = performance.now();
    swapBtn.classList.add("armed");
    swapBtn.textContent = "Click again to switch";
    setTimeout(() => { if (performance.now() - swapArmed >= 4000) disarm(); }, 4100);
    return;
  }
  swapArmed = -1e9;
  disarm();
  swapping = true;
  swapAt = performance.now();
  showSwapping();
  post([{ act: "swap" }]).then(async (r) => {
    if (r && !r.ok) {
      let why = "";
      try { why = (await r.json()).error || ""; } catch (e) {}
      note("Not switched: " + (why || "HTTP " + r.status));
    }
  });
});

const tog = $("tog");
function setBare(bare) {
  document.body.classList.toggle("bare", bare);
  tog.setAttribute("aria-expanded", String(!bare));
  try { localStorage.setItem("stereopsis.bare", bare ? "1" : "0"); } catch (e) {}
}
tog.addEventListener("click", (e) => { e.stopPropagation(); setBare(!document.body.classList.contains("bare")); });
try { if (localStorage.getItem("stereopsis.bare") === "1") setBare(true); } catch (e) {}
screen.addEventListener("click", () => screen.classList.toggle("nohud"));
document.addEventListener("keydown", (e) => {
  const t = e.target, tag = t && t.tagName;
  if (tag === "INPUT" || tag === "SELECT" || e.ctrlKey || e.metaKey || e.altKey || document.body.classList.contains("view")) return;
  const k = e.key.toLowerCase();
  if (k === "arrowright") { busy.mode = performance.now(); act({ act: "mode", step: 1 }); }
  else if (k === "arrowleft") { busy.mode = performance.now(); act({ act: "mode", step: -1 }); }
  else if (k === "t" || (k === " " && tag !== "BUTTON")) fire();
  else if (k === "o") act({ act: "osd" });
  else if (k === "p") act({ act: "persist" });
  else return;
  e.preventDefault();
});

v.onerror = () => {
  if (quiet()) overlay("waiting for the EYESY…");
  setTimeout(() => { v.src = "/stream.mjpg?" + Date.now(); }, 1500);
};
v.onload = () => { if (quiet()) $("down").style.display = "none"; };

async function loadModes() {
  if (loadingModes) return;
  loadingModes = true;
  try {
    const m = await (await fetch("/modes.json", { cache: "no-store" })).json();
    sel.replaceChildren(...m.modes.map((n) => { const o = document.createElement("option"); o.value = n; o.textContent = n; return o; }));
    modeCount = m.modes.length;
  } catch (e) {}
  loadingModes = false;
}
const spec = $("spec"), sctx = spec.getContext("2d");
let lastBeats = -1;
function drawSpec(an) {
  const dpr = window.devicePixelRatio || 1, w = Math.round(spec.clientWidth * dpr), h = Math.round(spec.clientHeight * dpr);
  if (spec.width !== w || spec.height !== h) { spec.width = w; spec.height = h; }
  sctx.clearRect(0, 0, w, h);
  const bpm = $("bpm");
  if (!an || !an.on) { bpm.textContent = "off"; return; }
  const n = an.bands.length, bw = w / n;
  sctx.fillStyle = "#8ab4f8";
  for (let i = 0; i < n; i++) {
    const bh = Math.max(1, an.bands[i] * h);
    sctx.fillRect(i * bw + dpr * 0.5, h - bh, Math.max(1, bw - dpr), bh);
  }
  bpm.textContent = an.bpm > 0 ? Math.round(an.bpm) + " BPM" : DASH;
  const beat = lastBeats >= 0 && an.beats !== lastBeats;      // a kick since the last look
  lastBeats = an.beats;
  bpm.classList.toggle("beat", beat || an.beat > 0.5);
}
function render(s) {
  $("mode").textContent = s.mode;
  curMode = s.mode;
  $("meta").textContent = [Math.round(s.fps) + " fps", "draws " + s.render[0] + TIMES + s.render[1],
    s.persist ? "persist" : "", s.scene || "", s.menu ? "menu open" : ""].filter(Boolean).join(DOT);
  document.body.classList.toggle("view", !s.controls);
  if (!s.controls) return;
  if (s.mode_count !== modeCount) loadModes();
  if (fresh("mode") && document.activeElement !== sel) sel.value = s.mode;
  knobs.forEach((k, i) => {
    if (fresh("k" + i)) { k.inp.value = s.knobs[i]; k.out.textContent = s.knobs[i].toFixed(2); }
    k.row.classList.toggle("held", s.held[i]);
    k.hw.style.setProperty("--p", s.hw[i]);
  });
  if (fresh("g")) { gain.inp.value = s.gain; gain.out.textContent = s.gain.toFixed(2); }
  drawSpec(s.an);
  $("osd").setAttribute("aria-pressed", String(s.osd));
  $("persist").setAttribute("aria-pressed", String(s.persist));
  $("scene").textContent = s.scene || (s.scene_count ? DASH : "no scenes saved");
  document.querySelectorAll(".sc").forEach((b) => { b.disabled = !s.scene_count; });
  $("fg").textContent = s.fg;
  $("bg").textContent = s.bg;
  $("release").disabled = !s.held.some(Boolean);
  $("swaprow").hidden = !s.swap;
  $("sizerow").hidden = !s.display;              // the display's size, half, a quarter: which one this mode draws at
  textrow.hidden = typeof s.text !== "string";
  if (typeof s.text === "string" && s.text !== shownText && document.activeElement !== textin && fresh("text")) {
    textin.value = s.text;
    shownText = s.text;
  }
  if (s.display) document.querySelectorAll(".size").forEach((b) => {
    const w = Math.floor(s.display[0] / SIZES[b.dataset.size]), h = Math.floor(s.display[1] / SIZES[b.dataset.size]);
    b.textContent = w + TIMES + h;
    b.setAttribute("aria-pressed", String(s.render[0] === w && s.render[1] === h));
  });
}
async function poll() {
  try {
    const s = await (await fetch("/state.json", { cache: "no-store" })).json();
    if (s.engine && s.engine !== "stereopsis") { location.reload(); return; }    // the stock engine's page is here now
    render(s);
    panel.classList.remove("off");
    if (swapping && performance.now() - swapAt > 8000)          // still here: the engine did not switch
      note("Not switched: stereopsis is still running (its log says why)");
    if (quiet()) $("down").style.display = "none";
  } catch (e) {
    $("mode").textContent = swapping ? "switching engines" : "no connection";
    $("meta").textContent = "";
    panel.classList.add("off");
    if (swapping) showSwapping();
  }
  setTimeout(poll, 250);
}
poll();
</script></body></html>
"""


def _unit(a, key):
    """a number from 0 to 1 (clamped); ValueError for anything that is not a finite number"""
    v = a.get(key)
    if isinstance(v, bool) or not isinstance(v, (int, float)):
        raise ValueError("%s must be a number from 0 to 1" % key)
    try:
        f = float(v)
    except OverflowError:
        raise ValueError("%s must be a number from 0 to 1" % key)
    if not math.isfinite(f):
        raise ValueError("%s must be a number from 0 to 1" % key)
    return min(1.0, max(0.0, f))


def _panel(v):
    """snapped to the front panel's steps (its knobs read raw / 1023), so the page can only set values the
    hardware can: modes were only ever tested with those (stock Football Scope fails at exactly 0.5)"""
    return round(v * 1023) / 1023.0


def _index(a, key, n):
    v = a.get(key)
    if isinstance(v, bool) or not isinstance(v, int) or not 0 <= v < n:
        raise ValueError("%s must be a whole number from 0 to %d" % (key, n - 1))
    return v


def _step(a):
    v = a.get("step")
    if isinstance(v, bool) or v not in (1, -1):
        raise ValueError("step must be 1 or -1")
    return int(v)


class Preview(object):
    def __init__(self, disp, eyesy, port=8081, controls=True, swap_ok=None):
        self.disp, self.eyesy, self.port = disp, eyesy, port
        self.controls = controls
        self.swap_ok = swap_ok                       # () -> whether switching to the stock engine can work here
        self.sched = None                            # main.py's frame Scheduler (0.8.1: its tier in /state.json)
        self.render_request = None                   # the page's Size buttons: "full" / "half" / "quarter" (0.9)
        self.mode_text = None                        # the text of the mode on screen, if it writes one (0.10)
        self.queue = collections.deque(maxlen=256)   # checked actions from the page, applied by apply()
        self.server = None
        self.viewers = 0

    def _swap_ok(self):
        try:
            return bool(self.swap_ok and self.swap_ok())
        except Exception:
            return False

    def _sched(self):
        """the current mode's frame scheduling (tier: jit / commit / ahead), or None when it is off (JIT = 0)"""
        try:
            return self.sched.brief() if self.sched is not None else None
        except Exception:
            return None

    def _palette_name(self, i):
        try:
            return str(self.eyesy.palettes[i]["name"])
        except Exception:
            return str(i)

    def state(self):
        ey = self.eyesy
        scene = None
        try:
            if ey.scene_index >= 0:
                scene = ey.scenes[ey.scene_index]["name"]
        except Exception:
            pass
        return {"engine": "stereopsis", "swap": bool(self.controls) and self._swap_ok(),
                "mode": str(ey.mode), "mode_index": int(ey.mode_index), "mode_count": len(ey.mode_names),
                "fps": float(getattr(ey, "fps", 0.0)),
                "knobs": [float(x) for x in ey.knob[:5]], "hw": [float(x) for x in ey.knob_hardware[:5]],
                "held": [bool(x) for x in ey.knob_override[:5]],
                "render": [int(ey.xres), int(ey.yres)], "version": str(ey.VERSION), "osd": bool(ey.show_osd),
                "display": [int(self.disp.w), int(self.disp.h)] if self.disp.comp else None,   # (the Size buttons)
                "menu": bool(ey.menu_mode), "persist": not bool(ey.auto_clear), "scene": scene,
                "scene_count": len(ey.scenes), "fg": self._palette_name(ey.fg_palette),
                "bg": self._palette_name(ey.bg_palette), "gain": float(ey.config.get("audio_gain", 0.0)),
                "controls": bool(self.controls), "viewers": self.viewers, "sched": self._sched(),
                "text": self.mode_text,
                "audio": [float(ey.audio_peak), float(ey.audio_peak_r)],
                "an": {"on": bool(getattr(ey, "audio_analysis", False)), "level": float(getattr(ey, "audio_level", 0)),
                       "bass": float(getattr(ey, "audio_bass", 0)), "mid": float(getattr(ey, "audio_mid", 0)),
                       "treble": float(getattr(ey, "audio_treble", 0)), "beat": float(getattr(ey, "audio_beat", 0)),
                       "beats": int(getattr(ey, "beat_count", 0)), "bpm": float(getattr(ey, "bpm", 0)),
                       "bands": [round(float(x), 3) for x in getattr(ey, "audio_bands", [])],
                       # diagnostics: window RMS (0..1), tempo confidence, onset activity (see audio.c)
                       "rms": float((getattr(ey, "_audio_vals", None) or [0] * 16)[9]),
                       "conf": float((getattr(ey, "_audio_vals", None) or [0] * 16)[8]),
                       "activity": float((getattr(ey, "_audio_vals", None) or [0] * 16)[12])},
                # running totals, for measuring (difference two readings): the compositor's frames, issued,
                # completed, errors, then ms spent on copy (+ preview grab), clear, OSD, flip wait, commit, engine
                # wait; the preview's grabs, encodes, encode errors, ms downscaling (+ OSD blend), ms encoding
                "perf": {"comp": self.disp.comp_stats()[:10], "preview": self.disp.preview_stats()[:5]}}

    def modes(self):
        return {"modes": [str(m) for m in list(self.eyesy.mode_names)]}

    def parse_action(self, a):
        """check one action from the page; returns what apply() will run, or raises ValueError.
          {"act": "knob", "i": 0-4, "v": 0-1}       hold knob i at v (until that knob is turned); v snaps
                                                    to the panel's 1/1023 steps, like gain below
          {"act": "release"} / {"act": "release", "i": 0-4}   hand the knobs (or one) back to the hardware
          {"act": "mode", "step": 1|-1} / {"act": "mode", "name": "S - ..."}
          {"act": "scene", "step": 1|-1}            {"act": "palette", "which": "fg"|"bg", "step": 1|-1}
          {"act": "osd"} / {"act": "persist"}       toggles, as the buttons
          {"act": "trigger"} / {"act": "grab"}      the Trigger button, a screen grab
          {"act": "gain", "v": 0-1, "save": true}   audio gain (Shift + knob 1); save writes config.json
          {"act": "swap"}                           switch to the stock engine on this mode (main.py swap_to_stock)
          {"act": "render", "size": "full"|"half"|"quarter"}   the mode on screen draws at the display's size, half
                                                    or a quarter of it, saved in scale.json (main.py set_render_size)
          {"act": "text", "value": "...", "mode": "..."}   the text of a mode that writes one (its set_text: up to 64
                                                    printable characters); "reset": true = the text in its file
                                                    (its TEXT); with "mode", only if that mode is on screen
        """
        if not isinstance(a, dict):
            raise ValueError("an action is a JSON object")
        act = a.get("act")
        if act == "knob":
            return ("knob", _index(a, "i", 5), _panel(_unit(a, "v")))
        if act == "release":
            return ("release", (_index(a, "i", 5),) if "i" in a else tuple(range(5)))
        if act == "mode":
            if "name" in a:
                name = a["name"]
                if not isinstance(name, str) or name not in self.eyesy.mode_names:
                    raise ValueError("no such mode")
                return ("mode_name", name)
            return ("mode_step", _step(a))
        if act == "scene":
            return ("scene_step", _step(a))
        if act == "palette":
            which = a.get("which")
            if which not in ("fg", "bg"):
                raise ValueError("which must be fg or bg")
            return ("palette", which, _step(a))
        if act in ("osd", "persist", "trigger", "grab"):
            return (act,)
        if act == "gain":
            return ("gain", _panel(_unit(a, "v")), a.get("save") is True)
        if act == "swap":
            if not self._swap_ok():
                raise ValueError("switching engines is off on this EYESY (SWAP = 0, or no Engine Lab)")
            return ("swap",)
        if act == "render":
            size = a.get("size")
            if size not in ("full", "half", "quarter"):
                raise ValueError("size must be full, half or quarter")
            if not self.disp.comp:
                raise ValueError("render sizes need the compositor (this EYESY runs without it)")
            return ("render", size)
        if act == "text":
            mode = a.get("mode")
            if mode is not None and not isinstance(mode, str):
                raise ValueError("mode must be a mode's name")
            if a.get("reset") is True:
                return ("text", None, mode)
            t = a.get("value")
            if not isinstance(t, str):
                raise ValueError("value must be text")
            t = "".join(ch for ch in t if ch.isprintable())[:64]
            if not t.strip():
                raise ValueError("the text is empty")
            return ("text", t, mode)
        raise ValueError("unknown act")

    def control(self, headers, body):
        """one POST /control: check it and queue it. Returns (HTTP status, reply)."""
        if not self.controls:
            return 403, {"ok": False, "error": "this EYESY's preview is view-only (CONTROLS=0)"}
        if headers.get("Content-Type", "").split(";")[0].strip().lower() != "application/json":
            return 415, {"ok": False, "error": "send JSON (Content-Type: application/json)"}
        origin = headers.get("Origin")
        if origin is not None and origin != "http://" + headers.get("Host", ""):
            return 403, {"ok": False, "error": "controls only take requests from this page"}
        try:
            msg = json.loads(body.decode("utf-8"))
            acts = msg if isinstance(msg, list) else [msg]
            if not 0 < len(acts) <= MAX_ACTIONS:
                raise ValueError("send 1 to %d actions" % MAX_ACTIONS)
            cmds = [self.parse_action(a) for a in acts]
        except ValueError as e:                  # also bad JSON and bad UTF-8
            return 400, {"ok": False, "error": str(e)[:200]}
        self.queue.extend(cmds)
        return 200, {"ok": True, "queued": len(cmds)}

    def apply(self):
        """the engine's main loop calls this once per frame, before it reads the knobs: every action the page
        queued since the last frame, in order, on the main thread"""
        q, ey = self.queue, self.eyesy
        while q:
            try:
                c = q.popleft()
            except IndexError:
                break
            try:
                k = c[0]
                if k == "knob":
                    ey.cc_override_knob(c[1], c[2])       # held until that knob moves, as MIDI CC does
                elif k == "release":
                    for i in c[1]:
                        ey.knob_override[i] = False
                        ey.knob[i] = ey.knob_hardware[i]
                elif k == "mode_step":
                    if c[1] > 0:
                        ey.next_mode()
                    else:
                        ey.prev_mode()
                elif k == "mode_name":
                    ey.set_mode_by_name(c[1])
                elif k == "scene_step":
                    if c[1] > 0:
                        ey.next_scene()
                    else:
                        ey.prev_scene()
                elif k == "palette":
                    if c[1] == "fg":
                        if c[2] > 0:
                            ey.next_fg_palette()
                        else:
                            ey.prev_fg_palette()
                    elif c[2] > 0:
                        ey.next_bg_palette()
                    else:
                        ey.prev_bg_palette()
                elif k == "osd":
                    ey.toggle_osd()
                elif k == "persist":
                    ey.toggle_auto_clear()
                elif k == "trigger":
                    ey.trig = True
                elif k == "grab":
                    ey.screengrab_flag = True
                elif k == "gain":
                    ey.config["audio_gain"] = c[1]
                    if c[2]:
                        ey.save_config_file()
                elif k == "swap":
                    ey.swap_request = "the page"       # the main loop switches right after this
                elif k == "render":
                    self.render_request = c[1]         # the main loop sets it right after this (set_render_size)
                elif k == "text":
                    m = sys.modules.get(ey.mode)
                    setter = getattr(m, "set_text", None)
                    if callable(setter) and (c[2] is None or c[2] == ey.mode):
                        got = setter(c[1] if c[1] is not None else str(getattr(m, "TEXT", "")))
                        print("[stereopsis] %s: text %r from the page" % (ey.mode, got), flush=True)
            except Exception as e:
                print("[stereopsis] preview control %r failed: %r" % (c, e), flush=True)
        self.mode_text = self._mode_text()

    def _mode_text(self):
        """the text of the mode on screen, if it writes one: module-level get_text() and set_text() (0.10)"""
        m = sys.modules.get(self.eyesy.mode)
        get, put = getattr(m, "get_text", None), getattr(m, "set_text", None)
        if not (callable(get) and callable(put)):
            return None
        try:
            t = get()
            return t if isinstance(t, str) else None
        except Exception:
            return None

    def next_jpeg(self, buf, seq, wait_s=0.25):
        """the next JPEG newer than seq, or None after wait_s"""
        end = time.time() + wait_s
        while True:
            n, seq2 = self.disp.preview_jpeg(buf, seq)
            if n > 0:
                return bytes(buf[:n]), seq2
            if n < 0 or time.time() >= end:
                return None, seq
            time.sleep(0.008)

    def start(self):
        pv = self

        class Handler(BaseHTTPRequestHandler):
            protocol_version = "HTTP/1.1"
            timeout = 60                  # a connection that sends or takes nothing for a minute is dropped

            def log_message(self, fmt, *args):        # keep the engine's log for the engine
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
                        self._send(200, "text/html; charset=utf-8", PAGE.encode())
                    elif path == "/state.json":
                        self._json(200, pv.state())
                    elif path == "/modes.json":
                        self._json(200, pv.modes())
                    elif path == "/frame.jpg":
                        buf = ctypes.create_string_buffer(1 << 20)
                        jpg, _ = pv.next_jpeg(buf, 0, wait_s=1.0)
                        if jpg is None:
                            self._send(503, "text/plain", b"no frame yet\n")
                        else:
                            self._send(200, "image/jpeg", jpg)
                    elif path == "/stream.mjpg":
                        self.stream()
                    else:
                        self._send(404, "text/plain", b"not found\n")
                except OSError:                              # the viewer went away or stalled (timeout)
                    self.close_connection = True
                except Exception as e:                       # a bug here must not take the engine's log with it
                    self.close_connection = True
                    print("[stereopsis] preview: GET %s failed: %r" % (path, e), flush=True)

            def do_POST(self):
                try:
                    try:
                        n = int(self.headers.get("Content-Length") or -1)
                    except ValueError:
                        n = -1
                    if n < 0 or n > MAX_BODY:
                        self.close_connection = True
                        if MAX_BODY < n <= DRAIN_MAX:        # read it first, so the reply is not lost to a reset
                            while n > 0:
                                got = self.rfile.read(min(n, 65536))
                                if not got:
                                    break
                                n -= len(got)
                            n = MAX_BODY + 1
                        return self._json(413 if n > MAX_BODY else 411, {"ok": False, "error": "body length"})
                    body = self.rfile.read(n) if n else b""
                    if self.path.split("?", 1)[0] != "/control":
                        return self._send(404, "text/plain", b"not found\n")
                    code, reply = pv.control(self.headers, body)
                    self._json(code, reply)
                except OSError:                              # the viewer went away or stalled (timeout)
                    self.close_connection = True
                except Exception as e:
                    self.close_connection = True
                    print("[stereopsis] preview: POST %s failed: %r" % (self.path[:80], e), flush=True)

            def stream(self):
                self.send_response(200)
                self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
                self.send_header("Cache-Control", "no-store")
                self.send_header("Connection", "close")
                self.end_headers()
                buf = ctypes.create_string_buffer(1 << 20)
                seq = 0
                pv.viewers += 1
                try:
                    started = time.time()
                    while time.time() - started < 12 * 3600:   # a forgotten tab stops after 12 h
                        jpg, seq = pv.next_jpeg(buf, seq)
                        if jpg is None:
                            continue
                        self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: %d\r\n\r\n" % len(jpg))
                        self.wfile.write(jpg)
                        self.wfile.write(b"\r\n")
                finally:
                    pv.viewers -= 1
                    self.close_connection = True

        class Server(ThreadingHTTPServer):
            daemon_threads = True
            allow_reuse_address = True

        try:
            self.server = Server(("0.0.0.0", self.port), Handler)
        except OSError as e:
            print("[stereopsis] browser preview: port %d unavailable (%s)" % (self.port, e), flush=True)
            return False
        threading.Thread(target=self.server.serve_forever, name="preview-http", daemon=True).start()
        print("[stereopsis] browser preview on http://<this EYESY's address>:%d/ (controls %s)"
              % (self.port, "on" if self.controls else "off: view-only"), flush=True)
        return True
