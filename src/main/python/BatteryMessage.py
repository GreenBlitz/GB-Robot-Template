import sys
import time
import tkinter as tk
from tkinter import PhotoImage
from NetworkTableManager import NetworkTableClient

__IMAGE_PATH = "noam-battery-message.png"
__WINDOW_NAME = "Battery Message"

__CLIENT_NAME = "BatteryMessage"
__TABLE_NAME = "Battery"
__KEY_NAME = "is low"
__IP = sys.argv[1]

__TIME_BETWEEN_MESSAGES_SECONDS = 4
__SHOW_MESSAGE_CHECK_COOLDOWN_SECONDS = 0.1


def __config_window(window: tk.Tk) -> None:
    window.title(__WINDOW_NAME)
    window.attributes("-topmost", True)
    window.resizable(False, False)
    window.bind("<Unmap>", lambda event: __cancel_minimize(event, window))


def __cancel_minimize(event, window: tk.Tk) -> None:
    window.attributes("-topmost", True)
    window.state('normal')


def __create_image_label(window: tk.Tk, image: PhotoImage) -> None:
    """Create a label widget to display the image on the given window."""
    label = tk.Label(window, image=image)
    label.pack()


def __load_image(image_path: str) -> PhotoImage:
    return PhotoImage(file=image_path)


def is_time_to_message(last_time_showed: float) -> bool:
    return time.time() - last_time_showed > __TIME_BETWEEN_MESSAGES_SECONDS


def __show_message() -> None:
    """Set up and run the Tkinter event loop."""
    window = tk.Tk()
    __config_window(window)
    image = __load_image(__IMAGE_PATH)
    __create_image_label(window, image)
    window.mainloop()


def __track_message_until_client_disconnect(battery_message_client: NetworkTableClient) -> None:
    battery_table = battery_message_client.get_table(__TABLE_NAME)

    last_time_showed = 0
    while battery_message_client.is_connected():
        if battery_table.getBoolean(__KEY_NAME, defaultValue=False) and is_time_to_message(last_time_showed):
            __show_message()
            last_time_showed = time.time()
        time.sleep(__SHOW_MESSAGE_CHECK_COOLDOWN_SECONDS)


def __run_battery_message_client() -> None:
    battery_message_client = NetworkTableClient(__IP, __CLIENT_NAME)
    battery_message_client.connect()
    __track_message_until_client_disconnect(battery_message_client)
    battery_message_client.terminate()


if __name__ == '__main__':
    __run_battery_message_client()
