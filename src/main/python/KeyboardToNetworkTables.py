import NetworkTableManager as nt
import sys
import ntcore
import keyboard
import time

KEYBOARD_CHECKING_COOLDOWN_SECONDS = 0.01
KEYBOARD_TABLE = "Keyboard"
KEYBOARD_KEYS_TABLE = "Keyboard/Keys"
CLIENT_NAME = "KeyboardToNetworkTables"
IP = sys.argv[1]


def is_pressed(event: keyboard.KeyboardEvent):
    return event.event_type == keyboard.KEY_DOWN


def on_key_event(event: keyboard.KeyboardEvent, table: ntcore.NetworkTable):
    if event is None or event.name is None:
        return
    elif event.name == "/":
        table.putBoolean("slash", is_pressed(event))
    elif event.is_keypad:
        table.putBoolean("numpad" + event.name, is_pressed(event))
    else:
        table.putBoolean(event.name.lower(), is_pressed(event))


def start_keyboard_tracking():
    network_table_instance = nt.get_connected_client(IP, CLIENT_NAME)
    table = network_table_instance.getTable(KEYBOARD_KEYS_TABLE)

    keyboard.hook(lambda key_event: on_key_event(key_event, table))
    while network_table_instance.isConnected():
        time.sleep(KEYBOARD_CHECKING_COOLDOWN_SECONDS)
    nt.terminate_client(network_table_instance, CLIENT_NAME)


if __name__ == "__main__":
    start_keyboard_tracking()