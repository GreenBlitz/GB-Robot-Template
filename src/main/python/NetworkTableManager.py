# Copyright (c) 2023 FRC 5990 Permission is hereby granted, free of charge, to any person obtaining a copy of this software and
# associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation
# the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit
# persons to whom the Software is furnished to do so, subject to the following conditions: The above copyright notice and this
# permission notice shall be included in all copies or substantial portions of the Software. THE SOFTWARE IS PROVIDED "AS IS",
# WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY
# CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION
# WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

# If this library is not installed, don't install ntcore but pyntcore.
import ntcore

import sys
import time
import logging

__CONNECTION_TIMEOUT_SECONDS = 30
__CONNECTION_COOLDOWN_SECONDS = 0.1

logging.basicConfig()
__LOGGER = (logging.getLogger("network table logger"))
__LOGGER.setLevel(logging.INFO)


def __past_connection_timeout(starting_time: float) -> bool:
    return time.time() - starting_time > __CONNECTION_TIMEOUT_SECONDS


def __wait_for_client_to_connect(network_table_instance: ntcore.NetworkTableInstance, client_name: str):
    starting_time = time.time()
    while not network_table_instance.isConnected():
        # terminate client and program if it takes to long to connect
        if __past_connection_timeout(starting_time):
            __LOGGER.error("Didn't connect to network tables. Terminating...")
            terminate_client(network_table_instance, client_name)
            sys.exit()
        time.sleep(__CONNECTION_COOLDOWN_SECONDS)


def get_connected_client(ip: str, client_name: str) -> ntcore.NetworkTableInstance:
    network_table_instance = ntcore.NetworkTableInstance.getDefault()

    __LOGGER.info("Setting up NetworkTables client named {}".format(client_name))
    network_table_instance.startClient4(client_name)
    network_table_instance.setServer(ip)
    network_table_instance.startDSClient()

    __LOGGER.info("Waiting for connection to NetworkTables server...")
    __wait_for_client_to_connect(network_table_instance, client_name)

    __LOGGER.info("Connected {} to NetworkTables server".format(client_name))
    return network_table_instance


def terminate_client(network_table_instance: ntcore.NetworkTableInstance, client_name: str):
    __LOGGER.warning("Terminating client named {}".format(client_name))
    ntcore.NetworkTableInstance.destroy(network_table_instance)
