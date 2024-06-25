package frc.utils.applicationsutils;

import frc.robot.constants.RobotConstants;

import javax.swing.*;

public class OnDeploy {

    public static void main(String[] args) {
        startComputerPrograms();
    }

    private static void startComputerPrograms() {
        if (RobotConstants.ENABLE_KEYBOARD) {
            CMDHandler.runPythonClass("keyboard_to_nt.py");
        }
    }

}
