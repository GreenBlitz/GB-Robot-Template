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
from ntcore import NetworkTableInstance, NetworkTable, Topic

import time
import logging


class NetworkTableClient:
    _CONNECTION_TIMEOUT_SECONDS = 30
    _CONNECTION_COOLDOWN_SECONDS = 0.1

    def __init__(self, ip: str, client_name: str):
        self._ip = ip
        self._client_name = client_name
        self._network_table_instance = NetworkTableInstance.getDefault()
        logging.basicConfig()
        self._logger = logging.getLogger(f"client: {self._client_name}")
        self._logger.setLevel(logging.INFO)

    def _is_connection_timeout_exceeded(self, starting_time: float) -> bool:
        return time.time() - starting_time > self._CONNECTION_TIMEOUT_SECONDS

    def _wait_for_client_to_connect(self) -> None:
        starting_time = time.time()
        while not self.is_connected():
            if self._is_connection_timeout_exceeded(starting_time):
                self._logger.error("Didn't connect to network tables")
                self.terminate()
                raise TimeoutError("Past Connection Timeout")
            time.sleep(self._CONNECTION_COOLDOWN_SECONDS)

    def get_table(self, table_name: str) -> NetworkTable:
        return self._network_table_instance.getTable(table_name)

    def get_topic(self, topic_name: str) -> Topic:
        return self._network_table_instance.getTopic(topic_name)

    def is_connected(self) -> bool:
        return self._network_table_instance.isConnected()

    def connect(self) -> NetworkTableInstance:
        self._logger.info("Setting up NetworkTables client")
        self._network_table_instance.setServer(self._ip)
        self._network_table_instance.startClient4(self._client_name)

        self._logger.info("Waiting for connection to NetworkTables server...")
        self._wait_for_client_to_connect()

        self._logger.info(f"Connected {self._client_name} to NetworkTables server")
        return self._network_table_instance

    def terminate(self) -> None:
        self._logger.warning("Terminating client")
        NetworkTableInstance.destroy(self._network_table_instance)
