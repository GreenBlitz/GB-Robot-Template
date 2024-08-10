import matplotlib.pyplot as plt
import numpy
import time
from pvrecorder import PvRecorder

audio_resolution = 1


def start():
    recorder = PvRecorder(device_index=-1, frame_length=32 * audio_resolution)
    try:
        recorder.start()
        start_time = time.time()
        values = []
        while True:
            frame = recorder.read()
            values.extend(frame)
    except KeyboardInterrupt:
        recorder.stop()
    finally:
        plot_sound(values)
        plot_frequencies(values)
        recorder.delete()


def plot_frequencies(values: list):
    frequencies = numpy.fft.fft(values)
    plt.figure(figsize=(8, 6))
    plt.plot(numpy.arange(0, 1, 1 / len(frequencies)), numpy.abs(frequencies), 'r')
    plt.xlabel('Freq (Hz)')
    plt.ylabel('Amplitude')
    plt.show()


def plot_sound(values: list):
    plt.figure(figsize=(8, 6))
    plt.plot(numpy.arange(0, 1, 1 / len(values)), values, 'r')
    plt.xlabel('Time (s)')
    plt.ylabel('Amplitude')
    plt.show()


if __name__ == "__main__":
    start()
