import numpy
from pvrecorder import PvRecorder
import time

audio_resolution = 1;

def start():
    recorder = PvRecorder(device_index=-1, frame_length= 32 * audio_resolution)
    try:
        recorder.start()
        start_time = time.time()
        values = {}
        while True:
            frame = recorder.read()
            values.__setitem__(time.time() - start_time,frame)
            # Do something ...
    except KeyboardInterrupt:
        recorder.stop()
    finally:
        print(values)
        recorder.delete()


if __name__ == "__main__":
    start()
