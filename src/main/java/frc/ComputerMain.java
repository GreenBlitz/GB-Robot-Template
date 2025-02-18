package frc;

import frc.controllers.keyboard.KeyboardController;
import frc.utils.CMDHandler;

import javax.swing.*;
import java.nio.file.Path;

/**
 * Unless you know what you are doing, do not rename this file because it's being used elsewhere.
 */
public class ComputerMain {

	public static void main(String... args) {
		startComputerPrograms(args[0]);
	}

	private static void startComputerPrograms(String connectedIP) {
		CMDHandler.runPythonClass(Path.of("BatteryMessage"), connectedIP);

		if (KeyboardController.ENABLE_KEYBOARD) {
			CMDHandler.runPythonClass(Path.of("KeyboardToNetworkTables"), connectedIP);
		}

	}

}
