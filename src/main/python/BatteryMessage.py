import sys
import time
import tkinter
from tkinter import PhotoImage
from NetworkTableManager import NetworkTableClient

IMAGE_PATH = "../../../noam-battery-message.png"
WINDOW_NAME = "Battery Message"

CLIENT_NAME = "BatteryMessage"
LOW_BATTERY_TOPIC_NAME = "BatteryMessage/LowBattery"

TIME_BETWEEN_MESSAGES_SECONDS = 60
SHOW_MESSAGE_CHECK_COOLDOWN_SECONDS = 0.1


def config_window(window: tkinter.Tk) -> None:
    window.attributes("-topmost", True)
    window.resizable(False, False)
    window.bind("<Unmap>", lambda event: cancel_minimize(event, window))


def cancel_minimize(event, window: tkinter.Tk) -> None:
    window.attributes("-topmost", True)
    window.state('normal')


def create_image_label(window: tkinter.Tk, image: PhotoImage) -> None:
    """Create a label widget to display the image on the given window."""
    label = tkinter.Label(window, image=image)
    label.pack()


def load_image(image_path: str) -> PhotoImage:
    return PhotoImage(file=image_path)


def show_message(window_name: str, image_path: str) -> None:
    """Set up and run the Tkinter event loop."""
    window = tkinter.Tk()
    window.title(window_name)
    config_window(window)
    image = load_image(image_path)
    create_image_label(window, image)
    window.mainloop()


def is_time_to_message(last_time_showed: float) -> bool:
    return time.time() - last_time_showed > TIME_BETWEEN_MESSAGES_SECONDS


def track_message_until_client_disconnect(battery_message_client: NetworkTableClient) -> None:
    is_low_battery = battery_message_client.get_topic(LOW_BATTERY_TOPIC_NAME).genericSubscribe()

    last_time_showed = 0
    while battery_message_client.is_connected():
        if is_low_battery.getBoolean(defaultValue=False) and is_time_to_message(last_time_showed):
            show_message(WINDOW_NAME, IMAGE_PATH)
            last_time_showed = time.time()
        time.sleep(SHOW_MESSAGE_CHECK_COOLDOWN_SECONDS)


def run_battery_message_client(ip: str) -> None:
    client = NetworkTableClient(ip, CLIENT_NAME)
    client.connect()
    track_message_until_client_disconnect(client)
    client.terminate()


if __name__ == '__main__':
    run_battery_message_client(sys.argv[1])
