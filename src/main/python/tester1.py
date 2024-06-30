import sys
import time

import network_table_manager

CLIENT_NAME = "tester1"
IP = sys.argv[1]

LOOPS_COOLDOWN_SECONDS = 0.1


def start():
    network_table_instance = network_table_manager.get_network_table(IP, CLIENT_NAME)

    while network_table_instance.isConnected():
        print("tester1 connected")
        time.sleep(LOOPS_COOLDOWN_SECONDS)

    network_table_manager.destroy_network_table_instance(network_table_instance)


if __name__ == '__main__':
    start()
