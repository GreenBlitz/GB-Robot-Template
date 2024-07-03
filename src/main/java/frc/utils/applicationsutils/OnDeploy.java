package frc.utils.applicationsutils;

import frc.robot.constants.RobotConstants;

import java.nio.file.Path;

public class OnDeploy {

    public static void main(String[] args) {
        startComputerPrograms(args);
    }

    private static void startComputerPrograms(String[] args) {
        if (RobotConstants.ENABLE_KEYBOARD) {
            runKeyboard(args);
        }
    }

    private static void runKeyboard(String[] args) {
        CMDHandler.runPythonClass(Path.of("KeyboardToNetworkTables"), args[0]);
    }

}
