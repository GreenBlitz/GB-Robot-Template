from NetworkTableManager import NetworkTableInstance, NetworkTable, NetworkTableClient

import sys
import time
from typing import Callable, TypeVar

from pynput import keyboard

SpecialKey = TypeVar('SpecialKey')
ASCIIKey = TypeVar('ASCIIKey')
UndefinedKey = TypeVar('UndefinedKey')

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


def keys_handler(table: NetworkTableInstance, is_pressed: bool) -> Callable:
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


def track_keyboard_until_client_disconnect(keys_table: NetworkTable, keyboard_client: NetworkTableClient):
    listener = keyboard.Listener(
        on_press=keys_handler(keys_table, True),
        on_release=keys_handler(keys_table, False),
    )
    listener.start()
    while True:
        if not keyboard_client.is_connected():
            listener.stop()
            break
        time.sleep(KEYBOARD_EVENT_CHECKING_COOLDOWN_SECONDS)


def run_keyboard_tracking_client():
    keyboard_client = NetworkTableClient(IP, CLIENT_NAME)
    keys_table = keyboard_client.connect().getTable(KEYBOARD_KEYS_TABLE)

    track_keyboard_until_client_disconnect(keys_table, keyboard_client)
    keyboard_client.terminate()


if __name__ == "__main__":
    run_keyboard_tracking_client()
