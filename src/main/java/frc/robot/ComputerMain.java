package frc.robot;

import frc.utils.CMDHandler;

import java.nio.file.Path;

/**
 *  Unless you know what you are doing, do not rename this file because it's being used elsewhere.
 */
public class ComputerMain {

    public static void main(String[] args) {
        startComputerPrograms(args);
    }

    private static void startComputerPrograms(String... args) {
        String connectedIP = args[0];
        CMDHandler.runPythonClass(Path.of("BatteryMessage"), connectedIP);
    }

}
