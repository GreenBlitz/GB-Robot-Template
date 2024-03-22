package frc.robot;

import frc.robot.constants.Ports;
import frc.utils.humaninterfacedevices.SmartJoystick;

public class JoysticksBindings {

    private static final SmartJoystick MAIN_JOYSTICK = new SmartJoystick(Ports.Joystick.MAIN);
    private static final SmartJoystick SECOND_JOYSTICK = new SmartJoystick(Ports.Joystick.SECOND);
    private static final SmartJoystick THIRD_JOYSTICK = new SmartJoystick(Ports.Joystick.THIRD);
    private static final SmartJoystick FOURTH_JOYSTICK = new SmartJoystick(Ports.Joystick.FOURTH);

    public static void configureBindings() {
        mainJoystickButtons();
        secondJoystickButtons();
        thirdJoystickButtons();
        fourthJoystickButtons();
    }

    private static void mainJoystickButtons() {
        SmartJoystick usedJoystick = MAIN_JOYSTICK;
        //bindings
    }

    private static void secondJoystickButtons() {
        SmartJoystick usedJoystick = SECOND_JOYSTICK;
        //bindings
    }

    private static void thirdJoystickButtons() {
        SmartJoystick usedJoystick = THIRD_JOYSTICK;
        //bindings
    }

    private static void fourthJoystickButtons() {
        SmartJoystick usedJoystick = FOURTH_JOYSTICK;
        //bindings
    }


}
