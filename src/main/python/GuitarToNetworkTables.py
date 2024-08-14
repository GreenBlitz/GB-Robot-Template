from NetworkTableManager import NetworkTableInstance, NetworkTable, NetworkTableClient

import numpy
import time
import sys
from pvrecorder import PvRecorder

audio_resolution = 32
__KEYBOARD_EVENT_CHECKING_COOLDOWN_SECONDS = 0.01
__GUITAR_TABLE = "Guitar/Keys"
__CLIENT_NAME = "GuitarToNetworkTables"
__PRESSED_VALUE = 2000000
__IP = sys.argv[1]


def track_guitar_until_client_disconnect(keys_table: NetworkTable, guitar_client: NetworkTableClient):
    recorder = PvRecorder(device_index=-1, frame_length=audio_resolution)
    try:
        recorder.start()
        values = []
        while guitar_client.is_connected():
            frame = recorder.read()
            if len(values) > audio_resolution * 16:
                del values[0:audio_resolution]
            values.extend(frame)
            map_to_keys(keys_table, values)
            time.sleep(__KEYBOARD_EVENT_CHECKING_COOLDOWN_SECONDS)
    except KeyboardInterrupt:
        recorder.stop()
    finally:
        recorder.delete()


def map_to_keys(keys_table: NetworkTable, values: list):
    frequencies = numpy.abs(numpy.fft.fft(values))
    #TODO logic


def is_frequncies_in_threshold(values: list, low_bound: int, high_bound: int) -> bool:
    for i in range(low_bound, high_bound):
        if values[i] > __PRESSED_VALUE:
            return True
    return False


def run_guitar_tracking_client():
    guitar_client = NetworkTableClient(__IP, __CLIENT_NAME)
    keys_table = guitar_client.connect().getTable(__GUITAR_TABLE)

    track_guitar_until_client_disconnect(keys_table, guitar_client)
    guitar_client.terminate()


if __name__ == "__main__":
    run_guitar_tracking_client()
