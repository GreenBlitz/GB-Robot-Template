package frc.robot;

import edu.wpi.first.wpilibj2.command.PrintCommand;
import frc.robot.constants.Ports;
import frc.utils.controllers.joysticks.SmartJoystick;
import frc.utils.controllers.keyboard.KeyboardController;

public class JoysticksBindings {

    private static final SmartJoystick MAIN_JOYSTICK = new SmartJoystick(Ports.JoystickDriverStationPorts.MAIN);

    private static final SmartJoystick SECOND_JOYSTICK = new SmartJoystick(Ports.JoystickDriverStationPorts.SECOND);

    private static final SmartJoystick THIRD_JOYSTICK = new SmartJoystick(Ports.JoystickDriverStationPorts.THIRD);

    private static final SmartJoystick FOURTH_JOYSTICK = new SmartJoystick(Ports.JoystickDriverStationPorts.FOURTH);

    private static final KeyboardController KEYBOARD_CONTROLLER = new KeyboardController();


    public static void configureBindings() {
        mainJoystickButtons();
        secondJoystickButtons();
        thirdJoystickButtons();
        fourthJoystickButtons();
        keyboardButtons();
    }

    private static void mainJoystickButtons() {
        SmartJoystick usedJoystick = MAIN_JOYSTICK;
        // bindings
    }

    private static void secondJoystickButtons() {
        SmartJoystick usedJoystick = SECOND_JOYSTICK;
        // bindings
    }

    private static void thirdJoystickButtons() {
        SmartJoystick usedJoystick = THIRD_JOYSTICK;
        // bindings
    }

    private static void fourthJoystickButtons() {
        SmartJoystick usedJoystick = FOURTH_JOYSTICK;
        // bindings
    }

    private static void keyboardButtons() {
        KeyboardController usedJoystick = KEYBOARD_CONTROLLER;
        usedJoystick.A.onTrue(new PrintCommand("A PRESSED"));
        usedJoystick.A.onFalse(new PrintCommand("A RELEASED"));
        usedJoystick.B.onTrue(new PrintCommand("B PRESSED"));
        usedJoystick.B.onFalse(new PrintCommand("B RELEASED"));
    }

}
