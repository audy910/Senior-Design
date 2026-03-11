import sounddevice as sd
import numpy as np

# generate a 1 second beep
fs = 48000
t = np.linspace(0, 1, fs)
tone = (np.sin(2 * np.pi * 440 * t) * 0.3 * 32767).astype(np.int16)

sd.play(tone, samplerate=fs, device='pulse')
sd.wait()