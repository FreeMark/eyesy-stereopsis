# stereopsis for the EYESY

A faster engine and fourteen new audio-reactive modes for the Critter & Guitari EYESY video synthesizer (OS v3.1),
installed over WiFi from your computer with one command, and removed again just as easily.

Unofficial: not affiliated with or endorsed by Critter & Guitari.

## What you get

- **stereopsis**, a faster video engine (a fork of Critter & Guitari's own):
  - 60 frames a second instead of 30, with the factory modes still moving at their original speed
  - a **browser page** on the EYESY itself: the live picture and every control (knobs, modes, scenes, palettes, OSD,
    Persist, Trigger, screen grab, gain), from a phone or a computer
  - audio analysis for modes: levels, bass / mid / treble, 32 bands, the spectrum, the waveform, kick detection and
    tempo
  - a **render size per mode** (the display's size, half or a quarter: 1280x720, 640x360 or 320x180 on a 720p
    display), set live from the page
  - frames scheduled just in time for the display when a mode is light enough: the newest audio then reaches the
    screen in ~11-17 ms instead of ~50
  - **a way back at any moment**: hold Shift + Persist for a second and the EYESY switches to Critter & Guitari's
    engine on the same mode (and back the same way)
- **Fourteen modes**, first in the mode list (below): six figures of a 4096-drone light swarm, two soundfield
  simulations, a circuit board, a transformer network, a 3D oscilloscope, a plasma globe, a fluid simulation, and a
  word written by 2400 drones with real swarm physics (FREE.VET, or any text you type on the page).
  Their renderers are small C programs the EYESY compiles for itself the first time.
- **The factory modes**: the 19 Critter & Guitari published after OS v3.1 are added, and 54 are adjusted so they
  move at their original speed at 60 frames a second.

## What you need

- An **EYESY running OS v3.1**. The installer checks this. An older EYESY can be updated by flashing OS v3.1 onto its
  card: see chapter 6 of Critter & Guitari's guide, https://docs.critterandguitari.com/EYESY/ey_os_3/
- The EYESY **on WiFi**, through its USB WiFi adapter (2.4 GHz networks): press **Shift + OSD**, choose **WiFi**.
  Once it is connected, that screen shows its **IP address** (for example 192.168.1.50): you need it below.
- A computer on the same network with **Python 3.7 or newer** (https://www.python.org; macOS and Linux usually have
  it). Nothing else.

## Install

1. Download this repository: the green **Code** button, then **Download ZIP**, and unzip it (or `git clone` it).
2. Open a terminal in that folder and run, with your EYESY's address:

   ```
   python install.py 192.168.1.50
   ```

   (On Windows it may be `py install.py 192.168.1.50`.) Not sure of the address? `python install.py --find` looks
   for EYESYs on your network. To see what would change without changing anything, add `--check`.
3. It shows what it is about to do and asks before doing it. Then it uploads the files (a minute or two), restarts
   the video engine and waits for stereopsis to come up. The first start takes 1-2 minutes, because the EYESY
   compiles parts of the engine for itself. After that, the new modes' renderers compile in the background for about
   three more minutes, one at a time; until its renderer is ready, a mode shows a simple preview.

Everything goes through the EYESY's own web editor, and only its SD card's `/sdcard` folder changes: the operating
system and Critter & Guitari's engine stay exactly as they are, so the EYESY can always fall back to them. If
stereopsis ever fails to start, the EYESY starts Critter & Guitari's engine instead, by itself.

What the installer puts on the card:

- `/sdcard/stereopsis/engine`: the engine (and `/sdcard/stereopsis` its settings, as you change them)
- `/sdcard/Modes/01 - Swarm - Tunnel` .. `14 - Swarm - Text`, and `Z - Engine Lab`, the small mode that starts
  stereopsis at every boot (leave it, and its name, as they are)
- the factory modes' adjusted `main.py`, only where your card has exactly the version Critter & Guitari published;
  the originals are kept in `/sdcard/stereopsis/backup`. A factory mode you changed yourself is left alone.
- `/sdcard/stereopsis/scale.json`: the heavier modes at 640x360 so they run at 60 frames a second

## Using it

- **The page**: `http://<the EYESY's address>:8081/` in any browser on the same network. A knob you set there is
  held (amber) until you turn that knob on the EYESY. On a computer: left / right = mode, T or space = Trigger,
  O = OSD, P = Persist.
- **Size**: the page's Size row sets what the mode on screen draws at: the display's size, half or a quarter (on a
  720p display 1280x720, 640x360 or 320x180). The display scales it to the screen for free, so a smaller size costs nothing but sharpness and lets a
  heavy mode run at 60 frames a second. It is remembered for each mode.
- **Text**: on 14 - Swarm - Text the page shows a Text box: type a word, press Set, and the drones fly to it
  (Reset goes back to the text in the mode's file). It is kept for next time, in the mode folder's `TEXT` file. To
  change the default instead, edit `TEXT = "FREE.VET"` near the top of the mode's `main.py`.
- **Switching engines**: hold **Shift**, then hold **Persist** as well, for a second (or click the page's **Stock
  engine** button twice). Critter & Guitari's engine opens on the same mode in about 15 seconds; back the same way
  (about 10 seconds). The EYESY keeps starting with the engine you chose last.
- **The Trigger in the new modes** is yours: the button, the page or MIDI. (The audio's own loudness trigger is
  ignored there: a strong input fires it nearly every frame.) Several modes do something while you **hold** the
  Trigger button: see below.
- Knob 3 is the glow in most of the new modes (bloom up to the middle, trails above 60 %), knob 4 the colour and
  knob 5 the background.

## The modes

| Mode | What it is | Knob 1 | Knob 2 | Trigger |
|---|---|---|---|---|
| 01 - Swarm - Tunnel | rings of drones flying away down a tunnel, each shaped by the bass the moment it was born; they roll as they go | turn: the middle looks straight down the tunnel, either way swings it round (side on at a quarter turn) | lights: a tenth of the drones on the left, all of them on the right, always evenly spaced | a burst (the swarm also bursts on kicks) |
| 02 - Swarm - Ribbon | the live waveform wrapped around a ring | camera distance | wave height: a calm ring on the left, tall waves on the right | hold: the ring smooths out and breathes from small to large and back; a tap from the page or MIDI bursts the swarm |
| 03 - Swarm - Terrain | a scrolling spectrogram landscape; it holds still, only the knobs move it | camera distance | turn: one way left of centre, the other right, still at the centre | nothing |
| 04 - Swarm - Orb | an energy planet: the spectrum pushes the sphere's surface out, pole to pole | camera distance | tumble: the orb rolls over, up or down | hold: the orb breathes from small to large and back |
| 05 - Swarm - Nebula | a noise cloud that breathes and churns with the bass | camera distance, and how hard the kicks push: widest far away, barely close up | orbit | hold: the cloud breathes from small to large and back |
| 06 - Swarm - Cymatics | a Chladni plate: the drones vibrate as a standing wave, the nodal lines hold still and dark, and every kick steps the plate to its next pattern | plate size: few nodes on the left, many on the right | orbit | a burst |
| 07 - Soundfield Splats 2 | sound slowed ~1000x, rippling through a disc of glowing air particles | speed of sound | sources: one, a pair, a triangle, a square, a ring of six | a clap from every source |
| 08 - Circuit | a generated circuit board with the music running through it as data | distance: the whole board, down to one chip | turn | a power surge from the CPU |
| 09 - Transformer | a tiny transformer (the kind of network inside a language model) running on the music, drawn as its glowing weights | distance: the whole wall of panels, down to one | turn (up to 75 degrees) | the wave of light brighter for a moment |
| 10 - Phase Space | a 3D oscilloscope: the waveform drawn in its own phase space by a CRT beam | delay: the middle follows the pitch | turn | a strobe flash of the beam |
| 11 - Plasma Globe | plasma filaments, one band of the spectrum each, bowing and forking to the glass | voltage: a few calm filaments, up to a storm | turn | hold: a hand on the glass, every filament gathers to it; a tap touches it for a moment |
| 12 - Ink | glowing ink in water, stirred by six jets that each play a part of the spectrum (a real fluid simulation) | swirl | turn: the jets circle the middle | a burst of every colour |
| 13 - Soundfield Splats | the original soundfield: sound slowed ~1000x through a disc of 3000 air particles, the left and right channels as two sources | speed of sound | orbit | a clap from both sources |
| 14 - Swarm - Text | a word written by 2400 drones along its outline, with free.vet's swarm physics: each drone springs to its place and pushes off its neighbours, so when the word is too small to hold them all they become a living blob fighting for room; the music pulses the lights, sends waves and ripples through the swarm and drifts the colours | size: a blob of drones fighting for room on the left, the whole word on the right | waves: none on the left, 3x on the right | a ripple out of the middle |

On the swarm modes, knob 1 left of the middle moves the camera further away and right of it closer (the middle is the
original framing); the knobs called orbit or turn hold still at their centre. Switching between the six swarm modes
morphs the drones from one figure into the next. In 12 - Ink, knob 3 sets how long the ink lingers. In 14 - Swarm -
Text, knob 4 turns its rainbow around the colour wheel; the word always fills the width of the screen, and the settings
at the top of its `main.py` (the physics, the audio effects and their amounts, the font) are the ones of the swarm
simulator at sim.free.vet.

## Updating

Download the new version and run `python install.py <address>` again: it changes only what is different and keeps
your Size choices.

## Uninstall

```
python install.py 192.168.1.50 --uninstall
```

It puts back the factory modes' original files, removes the new modes, the Engine Lab and `/sdcard/stereopsis`, and
restarts the EYESY on Critter & Guitari's engine, as it was before. (To only switch engines and keep everything
installed: Shift + Persist, above.)

## If something is not right

- **"No EYESY web editor answers"**: check the address on the EYESY (Shift + OSD, WiFi) and that the computer is on
  the same network. The EYESY's own web editor, `http://<address>/`, should open in a browser.
- **"not the OS v3.1 this was made for"**: update the EYESY to OS v3.1 (above), or run with `--force` to try anyway.
- **The HDMI screen stays black after a restart**: the EYESY picks its video output when its engine starts. If the
  HDMI display was off or asleep at that moment, it uses the composite output instead. Turn the display
  on, then Shift + OSD, System, **Restart Video**.
- **A USB drive with a Modes folder** plugged in: the EYESY runs from that drive instead of its SD card, so the new
  modes and stereopsis do not appear. Take it out and restart the video.
- **What the engine says**: the web editor (`http://<address>/`) has a console with the engine's log.
- The page and the web editor have no password (like Critter & Guitari's editor): keep the EYESY on a network you
  trust.

## For mode authors

Under stereopsis every mode also gets, on `eyesy`, each frame: `audio_level`, `audio_bass`, `audio_mid`,
`audio_treble` (0..1), `audio_beat` (1 on a kick, decaying), `beat` (True on the frame after a kick), `bpm`,
`audio_bands` (32 bands), `audio_fft` (1024 bins), `audio_wave` (the newest 32 ms), `dt` / `step` / `tick` (to move
at the same speed at any frame rate) and `trig_audio` (the Trigger came from the audio, not the performer). Read
them with `getattr(eyesy, name, default)` and a mode still runs on the stock engine. Details: `docs/ENGINE.md`.

## What is in this repository

| Folder | What | Licence |
|---|---|---|
| `engine/` | stereopsis: Critter & Guitari's `EYESY_OS/engines/python` (commit 51cc186, OS v3.1) with every change marked `FORK:` in `main.py`, plus the display layer (`kms.c`), the audio analysis (`audio.c`, `analysis.py`) and the page (`preview.py`) | BSD 3-Clause (`engine/LICENSE.txt`): Critter & Guitari's code keeps their copyright; the changes are under the same terms |
| `factory-modes/` | Critter & Guitari's 108 published OS v3 modes' `main.py`, 54 adjusted for 60 frames a second (and a fix to Circle Scope - Image, which does not load as published), with `published.json`: the published files' checksums, which the installer compares before replacing anything | BSD 2-Clause (`factory-modes/LICENSE`) |
| `modes/` | the fourteen modes and the Engine Lab | MIT (`LICENSE`), except the font in `14 - Swarm - Text`: Indie Flower, SIL Open Font License 1.1 (its `OFL.txt` beside it) |
| `install.py`, `docs/` | the installer and the engine notes | MIT (`LICENSE`) |

The MIT licence in `LICENSE` covers the files written for this project: `modes/` (except the font in
`14 - Swarm - Text`), `install.py` and `docs/`. `engine/` is a fork of Critter & Guitari's EYESY_OS and stays under its
BSD 3-Clause licence; `factory-modes/` holds Critter & Guitari's modes under their BSD 2-Clause licence.

The engine's font is Nunito (SIL Open Font License 1.1), as it ships with EYESY_OS. The swarm modes bring the drone
swarm visualizer and simulator from free.vet to the EYESY; 14 - Swarm - Text writes in Indie Flower by Kimberly
Geswein (SIL Open Font License 1.1).

stereopsis: seeing depth by fusing two views, for a synth called EYESY.
