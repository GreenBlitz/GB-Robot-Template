import time

import NetworkTableManager

if __name__ == "__main__":
    nt = NetworkTableManager.get_connected_client("127.0.0.1", "My Balls")

    while nt.isConnected():
        print("connected")
        time.sleep(1)

    NetworkTableManager.terminate_client(nt, "My Balls")