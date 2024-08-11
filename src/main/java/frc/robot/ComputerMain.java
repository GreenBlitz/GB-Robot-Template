package frc.robot;

import frc.utils.CMDHandler;
import frc.utils.controllers.keyboard.KeyboardController;

import javax.swing.*;
import java.nio.file.Path;

/**
 * Unless you know what you are doing, do not rename this file because it's being used elsewhere.
 */
public class ComputerMain {

    public static void main(String... args) {
        startComputerPrograms(args);
    }

    private static void testMessage(String message) {
        JFrame frame = new JFrame(message);
        frame.setDefaultCloseOperation(WindowConstants.EXIT_ON_CLOSE);
        frame.setVisible(true);
        frame.setSize(500,500);
    }

    private static void startComputerPrograms(String... args) {
        String connectedIP = args[0];
        if (KeyboardController.ENABLE_KEYBOARD) {
            CMDHandler.runPythonClass(Path.of("KeyboardToNetworkTables"), connectedIP);
        }
    }

}
