# Copyright (c) 2023 FRC 5990 Permission is hereby granted, free of charge, to any person obtaining a copy of this software and
# associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation
# the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit
# persons to whom the Software is furnished to do so, subject to the following conditions: The above copyright notice and this
# permission notice shall be included in all copies or substantial portions of the Software. THE SOFTWARE IS PROVIDED "AS IS",
# WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY
# CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION
# WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.


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