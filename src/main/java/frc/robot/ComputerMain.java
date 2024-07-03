package frc.robot;

import frc.robot.constants.RobotConstants;
import frc.utils.CMDHandler;

import javax.swing.*;
import java.nio.file.Path;
public class ComputerMain {

    public static void main(String[] args) {
//        startComputerPrograms(args);
        testMessage(args[0]);
    }

    private static void testMessage(String message) {
        JFrame frame = new JFrame();
        frame.setDefaultCloseOperation(WindowConstants.EXIT_ON_CLOSE);
        frame.setVisible(true);
        frame.setSize(500,500);
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
