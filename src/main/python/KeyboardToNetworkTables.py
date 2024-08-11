# Copyright (c) 2023 FRC 5990 Permission is hereby granted, free of charge, to any person obtaining a copy of this software and
# associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation
# the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit
# persons to whom the Software is furnished to do so, subject to the following conditions: The above copyright notice and this
# permission notice shall be included in all copies or substantial portions of the Software. THE SOFTWARE IS PROVIDED "AS IS",
# WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY
# CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION
# WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

from NetworkTableManager import NetworkTableInstance, NetworkTable, NetworkTableClient

import sys
import time
import keyboard


__KEYBOARD_EVENT_CHECKING_COOLDOWN_SECONDS = 0.01
__KEYBOARD_TABLE = "Keyboard"
__KEYBOARD_KEYS_TABLE = "Keyboard/Keys"
__CLIENT_NAME = "KeyboardToNetworkTables"
__IP = sys.argv[1]


def is_pressed(event: keyboard.KeyboardEvent) -> bool:
    return event.event_type == keyboard.KEY_DOWN


def on_key_event(event: keyboard.KeyboardEvent, table: NetworkTableInstance):
    if event is None or event.name is None:
        return
    elif event.name == "/":
        table.putBoolean("slash", is_pressed(event))
    elif event.is_keypad:
        table.putBoolean("numpad" + event.name, is_pressed(event))
    else:
        table.putBoolean(event.name.lower(), is_pressed(event))


def track_keyboard_until_client_disconnect(keys_table: NetworkTable, keyboard_client: NetworkTableClient):
    keyboard.hook(lambda key_event: on_key_event(key_event, keys_table))
    while keyboard_client.is_connected():
        time.sleep(__KEYBOARD_EVENT_CHECKING_COOLDOWN_SECONDS)


def run_keyboard_tracking_client():
    keyboard_client = NetworkTableClient(__IP, __CLIENT_NAME)
    keys_table = keyboard_client.connect().getTable(__KEYBOARD_KEYS_TABLE)

    track_keyboard_until_client_disconnect(keys_table, keyboard_client)
    keyboard_client.terminate()


if __name__ == "__main__":
    run_keyboard_tracking_client()
