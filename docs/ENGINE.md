# stereopsis: how the engine works

stereopsis is a fork of Critter & Guitari's `EYESY_OS/engines/python` (commit 51cc186, OS v3.1), BSD 3-Clause (see
`engine/LICENSE.txt`). It is not official and not endorsed by Critter & Guitari. Every change in `engine/main.py` is
marked `FORK:`; the files it adds are `kms.c` (the display layer), `display.py` (compiles and drives it), `audio.c`
and `analysis.py` (the audio analysis), `preview.py` (the page). The rest is upstream, with small changes to
`eyesy.py`, `osd.py`, `osc.py` and `sound.py`. The mode API is unchanged: every factory mode and every custom mode runs as it is.

## What it changes

| | Stock v3.1 | stereopsis |
|---|---|---|
| Presenting a frame | SDL uploads the whole frame as a GL texture through Mesa vc4 (~17 ms at 720p) | `kms.c`'s compositor thread copies the frame into scanout memory and clears it on a second core, then one atomic KMS commit |
| Render size | always the output size | per mode (`/sdcard/stereopsis/scale.json`), set live from the page's Size buttons; the display hardware scales it up for free |
| OSD | drawn into the frame (a full-frame copy each frame) | its own transparent hardware layer on top |
| Watching or playing it remotely | a capture card | a page on the EYESY itself: live picture and all the controls |
| Audio for modes | a 100-sample scope at 2 kHz and a peak | also levels, bass / mid / treble, 32 bands, a 1024-bin spectrum, the waveform, kick detection and tempo, analysed at 32 kHz on the audio process's core |
| Frame rate | 30 at most | 60 (paced by the display; `/sdcard/stereopsis/FPSCAP`; per mode `"@fps"` in scale.json) |
| Mode speed | a fixed step per frame, made for 30 fps | the same speed at 60 fps (below) |
| Going back | - | Shift + Persist held for a second (or the page's button) switches to the stock engine on the same mode, and back the same way |
| Audio to picture | ~50 ms | frames scheduled just in time for their vblank when a mode is light enough: the newest audio ~11-17 ms old on screen |

## The page: http://(the EYESY's address):8081/

Open it on a phone or a computer on the same network: the live picture (as HDMI shows it, OSD included) with the mode
and frame rate on top, and the EYESY's controls: the five knobs, the mode (step or pick from the list), scene,
foreground and background palette, OSD, Persist, Trigger, a screen grab, audio gain, Size, Text (for a mode that
writes one, below), and Stock engine.

- A knob set from the page is **held** (amber, with a tick where the real knob sits) until that knob on the EYESY is
  turned, the way a MIDI CC holds one; "Hand the knobs back" releases them all.
- Keys on a computer: left / right = mode, T or space = Trigger, O = OSD, P = Persist.
- The frames for the page are made on spare cores, only while someone watches: HDMI keeps its 60 frames a second.
- For scripts: `GET /state.json`, `GET /modes.json`, `GET /frame.jpg`, and `POST /control` with JSON actions (listed
  in `engine/preview.py`). `/control` takes only JSON and only from its own page's origin. There is no password,
  like the EYESY's own web editor: keep the EYESY on a network you trust. `/sdcard/stereopsis/CONTROLS` = 0 makes
  the page view-only; `PREVIEW` = 0 turns it off.

## For mode authors: a text the page can change

A mode that writes a text can let the page change it: give the module two functions, `get_text()` (the text on
screen) and `set_text(text)` (a new one; return what you set, or None to refuse). The page then shows a Text row for
that mode: the box shows `get_text()`, Set calls `set_text` with what was typed (up to 64 printable characters),
Reset calls it with the module's `TEXT`. Both run on the engine's thread, between frames (`get_text` once a frame,
for `/state.json` `"text"`, which is null for any other mode). Scripts: `{"act": "text", "value": "...",
"mode": "<its name>"}` or `{"act": "text", "reset": true}` on `/control`. 14 - Swarm - Text keeps the text in
its folder's `TEXT` file, so it survives a restart.

A second line (stereopsis 0.11): give the module `get_text2()` too (its second line, `""` while it shows one) and
`set_text2(text)` (`""` = one line again). The page then shows a Line 2 switch under the Text box: on shows a
second box (with the line it had before, sent at once, or empty to type in and Set), Set sends both lines, off goes
back to one line at once; Reset also calls `set_text2` with the module's `TEXT2`. `/state.json` has `"text2"` (null
for a mode without a second line); scripts: `{"act": "text", "value2": "..."}`, with or without `"value"`.
14 - Swarm - Text gives the second line drones of its own and keeps it in its `TEXT2` file (an empty one = one
line).

## For mode authors: the audio analysis

`engine/audio.c` runs in the engine's audio process on every block the sound card delivers (32 kHz; stock keeps 1
sample in 16 of it for `audio_in`). The spectrum and waveform follow the same gain as the scope data (Shift + knob 1),
but the analysis runs on the raw signal, so it never sees the clipping the gain causes. Every 16 ms it analyses the
last 64 ms exactly as a browser's AnalyserNode does (fftSize 2048, Blackman window, smoothing 0.6, -100..-30 dB), so
visuals written for Web Audio see the same numbers. On `eyesy`, every frame:

| Attribute | What |
|---|---|
| `audio_level`, `audio_bass`, `audio_mid`, `audio_treble` | 0..1: all / 20-200 Hz / 200-2000 / 2000-12000; fast to rise, slower to fall |
| `audio_beat` | 1 on a kick drum, decaying to 0 over ~0.3 s |
| `beat` | True on the first frame after a kick (like `trig`); `beat_count` counts them |
| `bpm`, `bpm_confidence` | the tempo of the last 8 s (0 = none yet: it needs 4 or more kicks in that window) |
| `audio_bands` | 32 log-spaced bands, 50 Hz - 16 kHz, 0..1 (a list) |
| `audio_fft` | the 1024-bin spectrum, 0..1, bin k = k * `audio_fft_hz` (15.625 Hz); a copy taken as the frame starts |
| `audio_wave` | the newest 32 ms, oldest first, -1..1; a copy taken as the frame starts |
| `audio_analysis` | True while the analyser runs |
| `trig_audio` | True when this frame's `trig` came only from the audio (the loudness trigger, or a kick with `TRIGGER` = beat / both), not from the Trigger button, MIDI or the page. The performer's Trigger: `eyesy.trig and not eyesy.trig_audio` (a hot input fires the loudness trigger nearly every frame). The button held down: `eyesy.key10_status` (stock) |

`audio_fft` and `audio_wave` are ctypes arrays (the same two objects every frame, steady while a mode draws): index
them from Python, or hand `ctypes.addressof(...)` to native code. The spectrum, bands and levels change with each 16
ms analysis; the waveform and beats with every 2 ms block. Kicks are found in the time domain: the raw signal through
a 150 Hz low-pass, its energy over the last 16 ms (looked at every 4 ms) twice its running average, a 0.12 s
refractory and a re-arm, so each kick counts once; a kick is reported ~6 ms after its onset on drums.
`/sdcard/stereopsis/TRIGGER` = `beat` (or `both`) lets kicks fire `eyesy.trig` for the trigger modes instead of the
loudness threshold; `ANALYSIS` = 0 turns the analysis off. It costs ~7 % of one core, on the audio process's core.

Read these with `getattr(eyesy, "audio_bass", 0.0)` and the like, and a mode still runs on the stock engine.

## Modes at 60 frames a second, at the speed they were made for

Stock draws at most 30 frames a second, and nearly every mode moves a fixed amount each frame: at 60 they would all
run twice as fast. stereopsis keeps the 60 and puts the speed back:

- **For every mode, in the engine.** The colour LFO and Python's `random` are replayed on the frames between two 30
  fps frames, so a mode that draws at random or cycles colours changes 30 times a second, exactly as on stock,
  however it uses them. The loudness trigger fires at most once per 30 fps frame.
- **For mode authors: `eyesy.dt`, `eyesy.step`, `eyesy.tick`.** `dt` = seconds since the previous frame (in whole
  display refreshes: no jitter), `step` = the same in 30 fps frames (0.5 at 60 fps, 1 on stock), `tick` = True on
  frames that complete a 30 fps frame. Multiply a per-frame amount by `step`; do a once-per-frame event (a push into
  a history, the next image) only when `tick`. Read them with `getattr(eyesy, "step", 1.0)` and the mode still runs
  unchanged on the stock engine.
- **The factory modes**: 54 of the 108 needed a change for this (`factory-modes/`); the installer puts them on the
  card only where the card has exactly the published file. Measured headless at 30 and at 60 fps with the same
  audio, knobs, randomness and triggers, 96 of the 108 draw exactly stock's picture at the same moments; the rest
  differ only where a bouncing or wrapping value lands (the speed is right) or are chaotic (Boids, Spinning Discs).

## Switching between stereopsis and the stock engine

Hold **Shift**, then hold **Persist** as well, for a second (Persist alone still toggles Persist; Shift + Persist did
nothing on stock). Or the page's **Stock engine** button, clicked twice (the first click arms it for 4 s). While the
stock engine runs, the same address shows a small page with the mode on screen and a **Switch to stereopsis** button.

- The other engine opens on the same mode, with the same Persist, colour and background palettes, OSD and gain.
  The knobs are the physical ones, so they carry over.
- It takes about 15 s to the stock engine and 10 s back: each engine sets every mode up before its first frame.
- **The EYESY keeps starting with the engine chosen last.** `/sdcard/stereopsis/SWAP` = 0 turns switching off.

How: stereopsis writes where the EYESY is to `/sdcard/stereopsis/RETURN`, sets the Engine Lab's boot switch to 0 and
exits; systemd starts the stock engine exactly as at power-on. Inside it, the Engine Lab opens the mode from RETURN,
watches for the hold and serves the small page; switching back, it writes RETURN, sets the boot switch to `boot` and
hands its process to stereopsis.

## How it starts (no reflashing)

The stock engine boots as usual. The mode `Z - Engine Lab` hands the running engine process over to stereopsis
(`os.execv`, same process, so systemd does not notice) when its switch file `STEREOPSIS` says so: `boot` = early in
every boot, `1` = when the lab mode is selected, `0` = never. Before handing over it writes
`/sdcard/stereopsis/.trial`; stereopsis deletes it after 300 good frames. If it dies before that, systemd restarts the
stock engine and the lab sets its switch back to 0, so a broken build can never lock the EYESY out.

On its first start stereopsis compiles `kms.c` and `audio.c` with the EYESY's own gcc (cached in
`/sdcard/stereopsis/engine/build`); the new modes compile their renderers the same way, in the background, one at a
time.

## Render size per mode

A mode draws into a surface of its render size, and the display hardware scales that to the screen at no cost: at
half size a mode touches a quarter of the pixels. The page's **Size** row shows the three sizes for the mode on
screen (the display's, half, a quarter) with the one it draws at lit; a click sets it from the next frame (the mode's
setup() runs again at that size) and saves it in `/sdcard/stereopsis/scale.json`:

```
{"default": "full", "modes": {"T - Woven Feedback": "640x360", "S - Boids": "full@30"}}
```

`"@fps"` runs that mode at a lower rate, locked to the display. The new modes draw at the full size of their surface on stereopsis (they read
the `STEREOPSIS` environment variable it sets); on the stock engine, whose surface is always the output size, they
draw at half of it for speed.

## Audio to picture

Each frame is scheduled per mode, from its last 90 frames' drawing time: **just in time** (the frame starts at its
target vblank minus drawing, the compositor's copy and a guard, so its audio is that young on screen: ~11-17 ms),
**after commit** (it starts as soon as the frame before is committed and shows two refreshes later: ~34 ms), or
**drawing ahead** (~50 ms, for modes whose drawing fills a refresh; the overlap is what holds their 60 frames a
second). Only a burst of misses drops a tier (5 within 10 s), and a dropped mode tries again after 30 s.
`/state.json` has the current tier under `"sched"`; `/sdcard/stereopsis/JIT` = 0 turns scheduling off. At 1280x720
the compositor's copy and clear take ~6 ms a frame, so the heavier modes draw ahead there and run just in time at
640x360. Not the engine's to remove: the spectrum's 64 ms window, the scan-out, and the display's own processing (a
TV's game mode matters).

## Setting modes up when first shown (off by default)

Stock imports every mode at boot and then runs every mode's `setup()`, which is why switching modes is seamless. With
`/sdcard/stereopsis/LAZY` = 1 each mode's `setup()` runs the first time it is shown instead: the engine starts in about
half the time with about half the memory, and the first show of each mode may pause briefly (most not visibly; modes
that load images longest).

## The switches in /sdcard/stereopsis

Files containing a value, read when the engine starts:

| File | What |
|---|---|
| `scale.json` | render size per mode (the page's Size row writes it) |
| `FPSCAP` | frame rate cap, default 60 |
| `TRIGGER` | what fires `eyesy.trig` from the audio: `peak` (stock's loudness threshold, the default), `beat` (kicks) or `both` |
| `ANALYSIS` | 0 = no audio analysis |
| `PREVIEW`, `PREVIEW_PORT`, `CONTROLS` | 0 = no page; its port (default 8081); 0 = a view-only page |
| `SWAP` | 0 = no switching engines |
| `JIT` | 0 = always draw ahead (no frame scheduling) |
| `LAZY` | 1 = set modes up when first shown |
| `METER` | 1 = log each mode's real frame rate and the compositor's timings |
| `LATENCY` | 1 = log how old the audio is when each frame starts and when it reaches the screen |

## Name, licence

stereopsis: seeing depth by fusing two views, for a synth called EYESY. Its own name, as the BSD licence's clause 3
asks. Critter & Guitari's code keeps their copyright and licence (`engine/LICENSE.txt`); stereopsis's additions are
under the same BSD 3-Clause terms. The font is Nunito (SIL Open Font License 1.1), as it ships with EYESY_OS.
