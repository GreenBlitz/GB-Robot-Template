from NetworkTableManager import NetworkTableInstance, NetworkTable, NetworkTableClient

import matplotlib.pyplot as plt
import numpy
import time
import sys
from pvrecorder import PvRecorder

audio_resolution = 1
__KEYBOARD_EVENT_CHECKING_COOLDOWN_SECONDS = 0.01
__GUITAR_TABLE = "Guitar/Keys"
__CLIENT_NAME = "GuitarToNetworkTables"
__IP = sys.argv[1]


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


def track_guitar_until_client_disconnect(keys_table: NetworkTable, guitar_client: NetworkTableClient):
    recorder = PvRecorder(device_index=-1, frame_length=32 * audio_resolution)
    try:
        recorder.start()
        start_time = time.time()
        values = []
        while guitar_client.is_connected():
            frame = recorder.read()
            values.extend(frame)
            time.sleep(__KEYBOARD_EVENT_CHECKING_COOLDOWN_SECONDS)
    except KeyboardInterrupt:
        recorder.stop()
    finally:
        recorder.delete()


def run_guitar_tracking_client():
    guitar_client = NetworkTableClient(__IP, __CLIENT_NAME)
    keys_table = guitar_client.connect().getTable(__GUITAR_TABLE)

    track_guitar_until_client_disconnect(keys_table, guitar_client)
    guitar_client.terminate()


if __name__ == "__main__":
    run_guitar_tracking_client()
