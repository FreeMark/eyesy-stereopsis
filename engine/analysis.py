"""analysis.py - stereopsis's audio analysis, the engine side (the analysing is audio.c, run by sound.py in the
engine's audio process on the full 32 kHz stream; stock only keeps a 2 kHz scope of it).

For modes, every frame (all 0..1 unless said otherwise):
  eyesy.audio_level, .audio_bass, .audio_mid, .audio_treble   loudness overall / 20-200 Hz / 200-2000 / 2000-12000,
                                                               fast to rise, slower to fall
  eyesy.audio_beat      1 on a kick drum, decaying to 0 over ~0.3 s
  eyesy.beat            True on the first frame after a kick (like eyesy.trig); eyesy.beat_count counts them
  eyesy.bpm             tempo estimate from the last 8 s (0 = none yet, or silence); eyesy.bpm_confidence
  eyesy.audio_bands     32 log-spaced bands, 50 Hz - 16 kHz (a list)
  eyesy.audio_analysis  True once the analysis runs (False: no analyser, e.g. it did not compile)
As ctypes arrays, copied at the start of each frame so they hold still while a mode draws (for native modes:
ctypes.addressof; the same arrays every frame):
  eyesy.audio_fft       1024 floats: the spectrum, bin k = k * eyesy.audio_fft_hz Hz (15.625 Hz at 32 kHz)
  eyesy.audio_wave      1024 floats: the newest 32 ms, oldest first, -1..1 (after the gain)
The spectrum is a browser AnalyserNode's (fftSize 2048, smoothing 0.6, -100..-30 dB), so code written for
Web Audio (getByteFrequencyData / 255) sees the same values. Everything follows the gain (Shift + knob 1).
How fresh (0.8): the spectrum, bands and levels come from an analysis every 16 ms; kicks (looked for every 4 ms) and
the waveform are published after every 2 ms block; a frame reads them all when it starts.
"""
import ctypes
import hashlib
import os
import platform
import subprocess
import time
from multiprocessing import Array, Value
from ctypes import c_float

HERE = os.path.dirname(os.path.abspath(__file__))
NVALS, NBANDS, NFFT, NWAVE = 16, 32, 1024, 1024
FLAGS = ["-O2", "-fno-math-errno", "-Wall", "-shared", "-fPIC"]
ARCH = ["-mcpu=cortex-a53", "-mfpu=neon-fp-armv8", "-mfloat-abi=hard"] if platform.machine().startswith("armv7") else []
AU_VERSION = 2


def _log(msg):
    print("[stereopsis] " + msg, flush=True)


def build():
    """compile (or reuse) audio.c, cached by a hash of source + flags; the ctypes library"""
    src_path = os.path.join(HERE, "audio.c")
    src = open(src_path, "rb").read()
    mach = platform.machine() or "unknown"
    tag = hashlib.sha1(src + " ".join(FLAGS + ARCH).encode()).hexdigest()[:10]
    name = "audio_%s_%s.so" % (tag, mach)
    dirs = [os.path.join(HERE, "build"), "/tmp"]
    so = next((os.path.join(d, name) for d in dirs if os.path.exists(os.path.join(d, name))), None)
    if so is None:
        t0 = time.time()
        for d in dirs:
            try:
                os.makedirs(d, exist_ok=True)
            except Exception:
                continue
            cand = os.path.join(d, name)
            tmp = cand + ".tmp%d" % os.getpid()
            r = subprocess.run(["gcc"] + FLAGS + ARCH + ["-o", tmp, src_path, "-lm"], capture_output=True, text=True,
                               timeout=240)
            if r.returncode == 0:
                os.replace(tmp, cand)
                so = cand
                _log("compiled %s in %.1f s" % (name, time.time() - t0))
                break
            _log("audio.c: gcc failed in %s: %s" % (d, r.stderr.strip()[-400:]))
        if so is None:
            raise RuntimeError("audio.c did not compile")
        for d in dirs:                                   # drop builds of older audio.c versions
            try:
                for f in os.listdir(d):
                    if f.startswith("audio_") and f.endswith(".so") and os.path.join(d, f) != so:
                        os.remove(os.path.join(d, f))
            except Exception:
                pass
    lib = ctypes.CDLL(so)
    lib.au_version.restype = ctypes.c_int
    lib.au_init.argtypes = [ctypes.c_int]
    lib.au_push.argtypes = [ctypes.c_char_p, ctypes.c_int, ctypes.c_int, ctypes.c_float]
    P = ctypes.POINTER(c_float)
    lib.au_results.argtypes = [P, ctypes.c_int, P, ctypes.c_int, P, ctypes.c_int, P, ctypes.c_int]
    if lib.au_version() != AU_VERSION:
        raise RuntimeError("audio.c version %d, expected %d" % (lib.au_version(), AU_VERSION))
    return lib


class Shared(object):
    """made by main.py before the audio process starts (so both processes share the memory); sound.py
    writes it, the engine's main loop reads it once per frame, both under the engine's audio lock"""

    def __init__(self):
        self.vals = Array(c_float, NVALS, lock=False)
        self.bands = Array(c_float, NBANDS, lock=False)
        self.fft = Array(c_float, NFFT, lock=False)
        self.wave = Array(c_float, NWAVE, lock=False)
        self.fft_frame = (c_float * NFFT)()          # the copies modes get: taken at the start of each frame, so they
        self.wave_frame = (c_float * NWAVE)()        # hold still while a mode draws (the shared ones change every 2 ms)
        # time.monotonic() (CLOCK_MONOTONIC, the same in both processes and in kms.c) of [0] the newest analysis
        # published, [1] the newest block read from the sound card; stat = 1: the audio process logs its reads
        self.times = Array(ctypes.c_double, 2, lock=False)
        self.stat = Value("i", 0, lock=False)
        self.last_seq = -1.0
        self.last_beats = None

    # ---- in the audio process (sound.py)
    def start_analyser(self, rate):
        """build + init audio.c in the audio process; None (and a log line) if that fails"""
        try:
            lib = build()
            if lib.au_init(int(rate)) != 0:
                raise RuntimeError("au_init(%d) refused" % rate)
            _log("audio analysis on: %d Hz, spectrum %d bins, %d bands, beat + tempo" % (rate, NFFT, NBANDS))
            return lib
        except Exception as e:
            _log("audio analysis off: %r" % (e,))
            return None

    def publish(self, lib):
        """after au_push ran an analysis: the results into the shared memory (hold the audio lock)"""
        lib.au_results(self.vals, NVALS, self.bands, NBANDS, self.fft, NFFT, self.wave, NWAVE)
        self.times[0] = time.monotonic()

    def publish_block(self, lib):
        """after any other block: the levels and beats (a kick is found every 4 ms) and the waveform (hold the lock)"""
        lib.au_results(self.vals, NVALS, None, 0, None, 0, self.wave, NWAVE)

    # ---- in the engine (main.py)
    def attach(self, eyesy):
        """the attributes modes use, at rest, before the first mode's setup()"""
        eyesy.audio_level = eyesy.audio_bass = eyesy.audio_mid = eyesy.audio_treble = 0.0
        eyesy.audio_beat = 0.0
        eyesy.beat = False
        eyesy.beat_count = 0
        eyesy.bpm = eyesy.bpm_confidence = 0.0
        eyesy.audio_bands = [0.0] * NBANDS
        eyesy.audio_fft = self.fft_frame
        eyesy.audio_wave = self.wave_frame
        eyesy.audio_fft_hz = 0.0
        eyesy.audio_analysis = False

    def update(self, eyesy, lock):
        """once per frame: the newest results onto eyesy (every frame: beats and the waveform change with every 2 ms
        block, not only with each analysis), and steady copies of the spectrum and waveform"""
        with lock:
            v = self.vals[:]
            b = self.bands[:]
            ctypes.memmove(self.fft_frame, self.fft, 4 * NFFT)
            ctypes.memmove(self.wave_frame, self.wave, 4 * NWAVE)
        self.last_seq = v[0]
        eyesy._audio_vals = v                        # everything audio.c reports (the page shows some)
        eyesy.audio_analysis = v[10] > 0.0
        eyesy.audio_level, eyesy.audio_bass, eyesy.audio_mid, eyesy.audio_treble, eyesy.audio_beat = v[1:6]
        beats = v[6]
        eyesy.beat = self.last_beats is not None and beats != self.last_beats
        if eyesy.beat:
            eyesy.beat_count += 1
        self.last_beats = beats
        eyesy.bpm, eyesy.bpm_confidence = v[7], v[8]
        eyesy.audio_fft_hz = v[10]
        eyesy.audio_bands = b
