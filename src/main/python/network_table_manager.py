# if this library is not installed, dont install ntcore but pyntcore
import ntcore
import sys
import time

TEAM_NUMBER = 4590  # GREENBLITZ 🐐🐐🐐🐐

CONNECTION_TIMEOUT_SECONDS = 30
LOOPS_COOLDOWN_SECONDS = 0.1


def get_network_table(ip: str, client_name: str):
    network_table_instance = ntcore.NetworkTableInstance.getDefault()

    print("Setting up NetworkTables client for team {}".format(TEAM_NUMBER))
    network_table_instance.startClient4(client_name)
    network_table_instance.setServer(ip)
    network_table_instance.startDSClient()

    print("Waiting for connection to NetworkTables server...")
    started_time = time.time()
    while not network_table_instance.isConnected():
        # terminate client and program if it takes to long to connect
        if time.time() - started_time > CONNECTION_TIMEOUT_SECONDS:
            print("Didn't connected to network tables. Terminating...")
            destroy_network_table_instance(network_table_instance)
            sys.exit()
        time.sleep(LOOPS_COOLDOWN_SECONDS)

    print("Connected NetworkTables server")
    return network_table_instance


def destroy_network_table_instance(network_table_instance: ntcore.NetworkTableInstance):
    # todo: add a way to print client name
    ntcore.NetworkTableInstance.destroy(network_table_instance)
