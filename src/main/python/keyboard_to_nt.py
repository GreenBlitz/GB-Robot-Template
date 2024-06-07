# Copyright (c) 2023 FRC 5990
# Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:
# The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

#Lets go Trigon

#if this library is not installed, dont install ntcore but pyntcore
import ntcore

import keyboard
import time

team = 4590  #GREENBLITZ 🐐🐐🐐🐐
client_name = "KeyboardToNT"
dashboard_server = "127.0.0.1"
connection_cooldown = 0.1
keyboard_table = "SmartDashboard/Keyboard"


def is_pressed(event: keyboard.KeyboardEvent):
    return event.event_type == keyboard.KEY_DOWN


def on_action(event: keyboard.KeyboardEvent, table: ntcore.NetworkTable):
    if event == None or event.name is None:
        return
    if event.name == "/":
        table.putBoolean("slash", is_pressed(event))
    if event.is_keypad:
        table.putBoolean("numpad" + event.name, is_pressed(event))
    else:
        table.putBoolean(event.name.lower(), is_pressed(event))


def get_table():
    network_table_instance = ntcore.NetworkTableInstance.getDefault()

    print("Setting up NetworkTables client for team {}".format(team))
    network_table_instance.startClient4(client_name)
    network_table_instance.setServer(dashboard_server)
    network_table_instance.startDSClient()

    # Wait for connection
    print("Waiting for connection to NetworkTables server...")
    while not network_table_instance.isConnected():
        time.sleep(connection_cooldown)

    table = network_table_instance.getTable(keyboard_table)
    table.putBoolean("Is Connected", True)
    return table


def main():
    table = get_table()

    keyboard.hook(lambda key_event: on_action(key_event, table))
    keyboard.wait()


if __name__ == '__main__':
    main()
