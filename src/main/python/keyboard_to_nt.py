# Copyright (c) 2023 FRC 5990
# Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:
# The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

import ntcore
import keyboard
import time

team = 5990


def main():
    def on_action(event: keyboard.KeyboardEvent):
        if event.name == "/" or event == None or event.name == None:
            return
        if event.is_keypad:
            table.putBoolean("numpad" + event.name, event.event_type == keyboard.KEY_DOWN)
        else:
            table.putBoolean(event.name.lower(), event.event_type == keyboard.KEY_DOWN)

    ntcoreinst = ntcore.NetworkTableInstance.getDefault()

    print("Setting up NetworkTables client for team {}".format(team))
    ntcoreinst.startClient4("KeyboardToNT")
    ntcoreinst.setServer("127.0.0.1")
    ntcoreinst.startDSClient()

    # Wait for connection
    print("Waiting for connection to NetworkTables server...")
    while not ntcoreinst.isConnected():
        time.sleep(0.1)

    print("Connected!")
    table = ntcoreinst.getTable("SmartDashboard/keyboard")

    keyboard.hook(lambda e: on_action(e))
    keyboard.wait()


if __name__ == '__main__':
    main()