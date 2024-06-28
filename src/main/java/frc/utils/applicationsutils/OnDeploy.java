package frc.utils.applicationsutils;

import frc.robot.constants.RobotConstants;

public class OnDeploy {

    public static void main(String[] args) {
        startComputerPrograms();
    }

    private static void startComputerPrograms() {
        runBatteryMessage();
    }

    private static void runBatteryMessage() {
        CMDHandler.runPythonClass("battery_message", RobotConstants.IS_RUNNING_ON_USB_B ? "172.22.11.2" : "10.45.90.2");
    }

}
