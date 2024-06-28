import sys
import time
import tkinter as tk
from tkinter import PhotoImage

# if this library is not installed, dont install ntcore but pyntcore
import ntcore

TEAM_NUMBER = 4590  # GREENBLITZ 🐐🐐🐐🐐

IMAGE_PATH = "noam-battery-message.png"
WINDOW_NAME = "Battery Message"

CLIENT_NAME = "BatteryMessage"
TABLE_NAME = "Battery"
KEY_NAME = "is low"
IP = sys.argv[1]

CONNECTION_COOLDOWN_SECONDS = 0.1
CONNECTION_TIMEOUT_SECONDS = 30
TIME_BETWEEN_MESSAGES_SECONDS = 4


def config_window(window):
    window.title(WINDOW_NAME)
    window.attributes("-topmost", True)
    window.resizable(False, False)
    window.bind("<Unmap>", lambda event: disable_minimize(event, window))


def disable_minimize(event, window):
    window.attributes("-topmost", True)
    window.state('normal')


def create_image_label(window, image):
    """Create a label widget to display the image on the given window."""
    label = tk.Label(window, image=image)
    label.pack()


def load_image(image_path):
    return PhotoImage(file=image_path)


def show_message():
    """Set up and run the Tkinter event loop."""
    window = tk.Tk()
    config_window(window)
    image = load_image(IMAGE_PATH)
    create_image_label(window, image)
    window.mainloop()


def get_network_table():
    network_table_instance = ntcore.NetworkTableInstance.getDefault()

    print("Setting up NetworkTables client for team {}".format(TEAM_NUMBER))
    network_table_instance.startClient4(CLIENT_NAME)
    network_table_instance.setServer(IP)
    network_table_instance.startDSClient()

    print("Waiting for connection to NetworkTables server...")
    started_time = time.time()
    while not network_table_instance.isConnected():
        # terminate client and program if it takes to long to connect
        if time.time() - started_time > CONNECTION_TIMEOUT_SECONDS:
            close_client(network_table_instance)
            sys.exit()
        time.sleep(CONNECTION_COOLDOWN_SECONDS)

    print("Connected NetworkTables server")
    return network_table_instance


def close_client(network_table_instance: ntcore.NetworkTableInstance):
    network_table_instance.stopDSClient()
    network_table_instance.stopClient()


def is_time_to_message(last_time_showed):
    return time.time() - last_time_showed > TIME_BETWEEN_MESSAGES_SECONDS


def start():
    network_table_instance = get_network_table()
    battery_table = network_table_instance.getTable(TABLE_NAME)  # todo: use less robust nt stuff then table

    last_time_showed = 0
    while network_table_instance.isConnected():
        if battery_table.getBoolean(KEY_NAME, defaultValue=False) and is_time_to_message(last_time_showed):
            show_message()
            last_time_showed = time.time()
        time.sleep(0.1)

    close_client(network_table_instance)


if __name__ == '__main__':
    start()
