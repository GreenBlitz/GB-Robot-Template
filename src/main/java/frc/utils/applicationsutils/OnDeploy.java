package frc.utils.applicationsutils;

import frc.robot.constants.RobotConstants;

import javax.swing.*;

public class OnDeploy {

    public static void main(String[] args) {
        startComputerPrograms();
    }

    private static void startComputerPrograms() {
        if (RobotConstants.ENABLE_KEYBOARD) {
            runKeyboard();
        }
    }

    private static void runKeyboard() {
        CMDHandler.runCMDCommand(
                "keyboard/keyboard_to_nt_" + (RobotConstants.IS_RUNNING_ON_USB ? "usb_b" : "real") + ".py"
        );
    }

}
