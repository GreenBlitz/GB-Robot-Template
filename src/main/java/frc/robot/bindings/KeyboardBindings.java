package frc.robot.bindings;

import edu.wpi.first.wpilibj2.command.PrintCommand;
import frc.utils.controllers.keyboard.KeyboardController;

public class KeyboardBindings {

    private static final KeyboardController KEYBOARD_CONTROLLER = new KeyboardController();

    public static void configureBindings() {
        KeyboardController usedKeyboard = KEYBOARD_CONTROLLER;
        usedKeyboard.A.whileTrue(new PrintCommand("A is pressed"));
        usedKeyboard.B.whileTrue(new PrintCommand("B is pressed"));
    }

}
