package frc.robot;

import frc.robot.constants.RobotConstants;
import frc.utils.CMDHandler;

import java.nio.file.Path;

public class ComputerMain {

    public static void main(String[] args) {
        startComputerPrograms();
    }

    private static void startComputerPrograms() {
        // start programs on computer...
        if (RobotConstants.ENABLE_KEYBOARD) {
            runKeyboard();
        }
    }

    private static void runKeyboard() {
        CMDHandler.runPythonClass(Path.of("keyboard_to_nt"), RobotConstants.IS_RUNNING_ON_USB_B ? "172.22.11.2" : "10.45.90.2");
    }

}
