from multiprocessing import Process, Array, Value
from ctypes import c_float
import alsaaudio
import struct
import pygame
import time

BUFFER_SIZE = 100  # Size of the circular buffer
max_peak = 0
max_peak_r = 0

def audio_processing(shared_buffer, shared_buffer_r, write_index, gain, peak, peak_r, lock, analysis=None):
    # FORK: analysis = stereopsis's analysis.Shared (audio.c gets every full-rate block; None = stock)
    global max_peak, max_peak_r
    
    def find_alsa_card_index(target_name="audioinjector-pi-soundcard"):
        """Finds the correct ALSA hardware index using card names."""
        num_cards = alsaaudio.card_indexes()  # Gets actual ALSA internal indexes
        for index in num_cards:
            card_name = alsaaudio.card_name(index)[1]  # Extract name from (index, name) tuple
            print(f"card {index} : {card_name}")
            if target_name in card_name.lower():
                return index
        raise ValueError(f"Sound card '{target_name}' not found!")

    # Try to find the correct card index dynamically
    try:
        card_index = find_alsa_card_index()
        print(f"Using ALSA card index: {card_index}")
    except ValueError as e:
        print(e)
        exit(1)

    # PCM setup
    channels = 1
    format = alsaaudio.PCM_FORMAT_S16_LE
    period_size = 32
    rate = 32000  # Adjust as needed
    bytes_per_sample = 2  # S16_LE = 2 bytes per sample

    # FORK: build the analyser before the card is opened (a first compile takes seconds)
    an_lib = analysis.start_analyser(rate) if analysis is not None else None

    # Open PCM using the correct index
    # FORK: 8 periods of room (16 ms) instead of the default 4 (8 ms): the reads still take each 2 ms period as it
    # arrives (no added delay), but a stall of this process while the engine's threads fill the cores (measured up to
    # 7.3 ms with a two-thread mode) no longer comes near an overrun. Older pyalsaaudio without `periods`: as stock
    try:
        pcm = alsaaudio.PCM(type=alsaaudio.PCM_CAPTURE, mode=alsaaudio.PCM_NORMAL, cardindex=card_index,
                            channels=channels, rate=rate, format=format, periodsize=period_size, periods=8)
    except TypeError:
        pcm = alsaaudio.PCM(
            type=alsaaudio.PCM_CAPTURE,
            mode=alsaaudio.PCM_NORMAL,
            cardindex=card_index,  # Now using the correct ALSA index
            channels=channels,
            rate=rate,
            format=format,
            periodsize=period_size
        )

    # Print ALSA PCM configuration
    print(pcm.dumpinfo())
    
    # FORK: LATENCY (analysis.stat): every 10 s, how the reads went - a read that returns at once means blocks were
    # already waiting (we are behind); the card's buffer holds 8 periods of 2 ms, so a longer stall is an overrun
    stat = analysis is not None and analysis.stat.value == 1
    st = {"t": time.monotonic(), "n": 0, "bad": 0, "at_once": 0, "gap": 0.0, "work": 0.0, "last": None}
    try:
        while True:
            t_ask = time.monotonic()
            length, data = pcm.read()
            if analysis is not None:
                t_got = time.monotonic()
                analysis.times[1] = t_got              # FORK: the newest block's arrival (LATENCY)
                if stat:
                    st["n"] += 1
                    st["bad"] += length != 64
                    st["at_once"] += t_got - t_ask < 0.0002
                    if st["last"] is not None:
                        st["gap"] = max(st["gap"], t_got - st["last"])
                        st["work"] = max(st["work"], t_ask - st["last"])
                    st["last"] = t_got
                    if t_got - st["t"] >= 10.0:
                        print("[stereopsis] LATENCY audio reads: %.0f/s, %d not 64 frames (overruns, errors), %d "
                              "returned at once, longest gap %.1f ms, longest work between reads %.1f ms" % (
                                  st["n"] / (t_got - st["t"]), st["bad"], st["at_once"], 1000 * st["gap"],
                                  1000 * st["work"]), flush=True)
                        st.update(t=t_got, n=0, bad=0, at_once=0, gap=0.0, work=0.0)
            #print(f"{length}, {len(data)}")
            if length == 64:
                # FORK: the analysis gets the whole block at 32 kHz (the scope data below keeps 1 in 16). After every
                # block the engine gets the levels, beats and the waveform (so a kick, found every 4 ms, and the
                # newest samples are never more than a block old); the spectrum after each analysis (every 16 ms)
                if an_lib is not None and len(data) >= length * 4:
                    ran = an_lib.au_push(data, length, 2, gain.value)
                    if ran >= 0:
                        with lock:
                            if ran > 0:
                                analysis.publish(an_lib)
                            else:
                                analysis.publish_block(an_lib)

                # Calculate the number of samples
                num_samples = length * bytes_per_sample

                # Unpack binary data into 16-bit signed integers
                samples = struct.unpack(f'<{num_samples}h', data)
                
                samples_l = samples[0::2]
                samples_r = samples[1::2]

                #print(len(samples))
                # Write samples to the circular buffer
                for i in range(0, len(samples_l), 16):
                    if i + 16 <= len(samples_l):
                        avg_sample = sum(samples_l[i:i+16]) / 16
                        avg_sample_r = sum(samples_r[i:i+16]) / 16

                        # Apply gain
                        avg_sample *= gain.value
                        avg_sample_r *= gain.value

                        # Clamp value to avoid overflow
                        avg_sample = max(-32768, min(32767, avg_sample))
                        avg_sample_r = max(-32768, min(32767, avg_sample_r))

                        # Check peak
                        if avg_sample > max_peak:
                            max_peak = avg_sample 
                        if avg_sample_r > max_peak_r:
                            max_peak_r = avg_sample_r  

                        # Lock only during shared variable writes
                        with lock:
                            shared_buffer[write_index.value] = avg_sample
                            shared_buffer_r[write_index.value] = avg_sample_r
                            write_index.value = (write_index.value + 1) % BUFFER_SIZE

                            # Update peak once per buffer cycle
                            if write_index.value == 0:
                                peak.value = max_peak
                                peak_r.value = max_peak_r
                                max_peak = 0  # Reset after committing
                                max_peak_r = 0  # Reset after committing


    finally:
        pcm.close()


