import sys
import time
from typing import Callable, TypeVar

from pynput import keyboard

from NetworkTableManager import NetworkTableInstance, NetworkTable, NetworkTableClient

Key = TypeVar("Key")

SpecialKey = TypeVar("SpecialKey", bound=Key)
ASCIIKey = TypeVar("ASCIIKey", bound=Key)
UndefinedKey = TypeVar("UndefinedKey", bound=Key)

KEYBOARD_EVENT_CHECKING_COOLDOWN_SECONDS = 0.05
KEYBOARD_KEYS_TABLE = "Keyboard/Keys"
CLIENT_NAME = "KeyboardToNetworkTables"


def get_key_type(key: Key) -> TypeVar:
    if hasattr(key, "name"):
        return SpecialKey
    elif hasattr(key, "char"):
        return ASCIIKey
    return UndefinedKey


def get_key_network_table_updater(table: NetworkTableInstance, is_pressed: bool) -> Callable:
    # Function to update the table based on the key type and its state (pressed or not).
    def update_network_table_key_state(key) -> None:
        key_type = get_key_type(key)  # Store the result to avoid repeated calls.
        if key_type is SpecialKey:
            table.putBoolean(key.name, is_pressed)
        elif key_type is ASCIIKey:
            if key.char == "/":
                table.putBoolean("slash", is_pressed)
            else:
                table.putBoolean(key.char.lower(), is_pressed)
    return update_network_table_key_state


def wait_until_client_disconnect(keyboard_client: NetworkTableClient) -> None:
    while True:
        if not keyboard_client.is_connected():
            break
        time.sleep(KEYBOARD_EVENT_CHECKING_COOLDOWN_SECONDS)


def create_keyboard_listener(keys_table: NetworkTable) -> keyboard.Listener:
    return keyboard.Listener(
        on_press=get_key_network_table_updater(keys_table, True),
        on_release=get_key_network_table_updater(keys_table, False),
    )


def track_keyboard_until_client_disconnect(keys_table: NetworkTable, keyboard_client: NetworkTableClient) -> None:
    keyboard_listener = create_keyboard_listener(keys_table)
    keyboard_listener.start()
    wait_until_client_disconnect(keyboard_client)
    keyboard_listener.stop()


def run_keyboard_tracking_client(ip: str) -> None:
    keyboard_client = NetworkTableClient(ip, CLIENT_NAME)
    keyboard_client.connect()
    keys_table = keyboard_client.get_table(KEYBOARD_KEYS_TABLE)

    track_keyboard_until_client_disconnect(keys_table, keyboard_client)
    keyboard_client.terminate()


if __name__ == "__main__":
    run_keyboard_tracking_client(sys.argv[1])