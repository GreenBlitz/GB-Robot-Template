package frc.robot;

import edu.wpi.first.wpilibj2.command.PrintCommand;
import frc.utils.controllers.keyboard.KeyboardController;

public class KeyboardBindings {

    private static final KeyboardController KEYBOARD_CONTROLLER = new KeyboardController();

    public static void configurateBindings() {
        keyboardButtons();
    }

    private static void keyboardButtons() {
        KeyboardController usedJoystick = KEYBOARD_CONTROLLER;
        usedJoystick.A.onTrue(new PrintCommand("A PRESSED"));
        usedJoystick.A.onFalse(new PrintCommand("A RELEASED"));
        usedJoystick.B.onTrue(new PrintCommand("B PRESSED"));
        usedJoystick.B.onFalse(new PrintCommand("B RELEASED"));
    }

}
