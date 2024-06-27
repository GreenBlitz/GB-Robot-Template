package frc.utils.applicationsutils;

import frc.robot.constants.RobotConstants;

import javax.swing.*;

public class OnDeploy {

    public static void main(String[] args) {
        startComputerPrograms();
    }

    private static void startComputerPrograms() {
        runBatteryMessage();
    }

    private static void runBatteryMessage(){
        CMDHandler.runPythonClass("battery/battery_message" + (RobotConstants.IS_RUNNING_ON_USB_B ? "usb_b" : "router") + ".py");
    }

}
