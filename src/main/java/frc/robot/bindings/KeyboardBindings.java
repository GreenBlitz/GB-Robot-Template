package frc.robot.bindings;

import edu.wpi.first.wpilibj2.command.PrintCommand;
import frc.utils.controllers.keyboard.KeyboardController;

public class KeyboardBindings {

    private static KeyboardController KEYBOARD_CONTROLLER;

    public static void configureBindings() {
        KEYBOARD_CONTROLLER = new KeyboardController();

        keyboardButtons();
    }

    private static void keyboardButtons() {
        KeyboardController usedKeyboard = KEYBOARD_CONTROLLER;
        usedKeyboard.A.onTrue(new PrintCommand("A PRESSED"));
        usedKeyboard.A.onFalse(new PrintCommand("A RELEASED"));
        usedKeyboard.B.onTrue(new PrintCommand("B PRESSED"));
        usedKeyboard.B.onFalse(new PrintCommand("B RELEASED"));
    }

}
