import sys
import time
from typing import Callable, TypeVar

from pynput import keyboard

from NetworkTableManager import NetworkTableInstance, NetworkTable, NetworkTableClient

SpecialKey = TypeVar("SpecialKey")
ASCIIKey = TypeVar("ASCIIKey")
UndefinedKey = TypeVar("UndefinedKey")

KEYBOARD_EVENT_CHECKING_COOLDOWN_SECONDS = 0.01
KEYBOARD_KEYS_TABLE = "Keyboard/Keys"
CLIENT_NAME = "KeyboardToNetworkTables"
IP = sys.argv[1]


def key_type_checker(key: object) -> TypeVar:
    if hasattr(key, "name"):
        return SpecialKey
    elif hasattr(key, "char"):
        return ASCIIKey
    return UndefinedKey


def keys_event_functions_factory(table: NetworkTableInstance, is_pressed: bool) -> Callable:
    # key type is changing in runtime + depends on platform. Checked for Xorg keyboard layout.
    def update_table(key) -> None:
        if key_type_checker(key) is UndefinedKey:
            return
        elif key_type_checker(key) is SpecialKey:
            table.putBoolean(key.name, is_pressed)
        elif key_type_checker(key) is ASCIIKey:
            if key.char == "/":
                table.putBoolean("slash", is_pressed)
            else:
                character: str = key.char
                table.putBoolean(character.lower(), is_pressed)

    return update_table


def wait_until_client_disconnect(keyboard_client: NetworkTableClient):
    while True:
        if not keyboard_client.is_connected():
            break
        time.sleep(KEYBOARD_EVENT_CHECKING_COOLDOWN_SECONDS)


def listener_factory(keys_table: NetworkTable) -> keyboard.Listener:
    return keyboard.Listener(
        on_press=keys_event_functions_factory(keys_table, True),
        on_release=keys_event_functions_factory(keys_table, False),
    )


def track_keyboard_until_client_disconnect(
    keys_table: NetworkTable, keyboard_client: NetworkTableClient
):
    keyboard_listener = listener_factory(keys_table)
    keyboard_listener.start()
    wait_until_client_disconnect(keyboard_client)
    keyboard_listener.stop()


def run_keyboard_tracking_client():
    keyboard_client = NetworkTableClient(IP, CLIENT_NAME)
    keys_table = keyboard_client.connect().getTable(KEYBOARD_KEYS_TABLE)

    track_keyboard_until_client_disconnect(keys_table, keyboard_client)
    keyboard_client.terminate()


if __name__ == "__main__":
    run_keyboard_tracking_client()
