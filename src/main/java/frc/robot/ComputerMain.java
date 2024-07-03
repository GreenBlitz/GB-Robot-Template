package frc.robot;

import frc.robot.constants.RobotConstants;
import frc.utils.CMDHandler;

import java.nio.file.Path;
public class ComputerMain {

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
    private static void startComputerPrograms() {
        // start programs on computer...
    }

}
