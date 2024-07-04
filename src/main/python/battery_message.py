# if this library is not installed, dont install ntcore but pyntcore
import ntcore
import sys
import time
import tkinter as tk
from tkinter import PhotoImage
import NetworkTableManager

__IMAGE_PATH = "noam-battery-message.png"
__WINDOW_NAME = "Battery Message"
__CLIENT_NAME = "BatteryMessage"
__TABLE_NAME = "Battery"
__KEY_NAME = "is low"
__IP = sys.argv[1]

__TIME_BETWEEN_MESSAGES_SECONDS = 180
__LOOPS_COOLDOWN_SECONDS = 0.1


def __config_window(window: tk.Tk):
    window.title(__WINDOW_NAME)
    window.attributes("-topmost", True)
    window.resizable(False, False)
    window.bind("<Unmap>", lambda event: __cancel_minimize(event, window))


def __cancel_minimize(event, window: tk.Tk):
    window.attributes("-topmost", True)
    window.state('normal')


def __create_image_label(window: tk.Tk, image: PhotoImage):
    """Create a label widget to display the image on the given window."""
    label = tk.Label(window, image=image)
    label.pack()


def __load_image(image_path: str) -> PhotoImage:
    return PhotoImage(file=image_path)


def __should_show_message(battery_table: ntcore.NetworkTable, last_time_showed: float) -> bool:
    is_time_to_message = time.time() - last_time_showed > __TIME_BETWEEN_MESSAGES_SECONDS
    is_battery_low = battery_table.getBoolean(__KEY_NAME, defaultValue=False)
    return is_battery_low and is_time_to_message


def __show_message():
    """Set up and run the Tkinter event loop."""
    window = tk.Tk()
    __config_window(window)
    image = __load_image(__IMAGE_PATH)
    __create_image_label(window, image)
    window.mainloop()


def start():
    battery_message_client = NetworkTableManager.get_connected_client(__IP, __CLIENT_NAME)
    battery_table = battery_message_client.getTable(__TABLE_NAME)

    last_time_showed = 0
    while battery_message_client.isConnected():
        if __should_show_message(battery_table, last_time_showed):
            __show_message()
            last_time_showed = time.time()
        time.sleep(__LOOPS_COOLDOWN_SECONDS)

    NetworkTableManager.terminate_client(battery_message_client, __CLIENT_NAME)


if __name__ == '__main__':
    start()
