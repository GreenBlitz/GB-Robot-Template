package frc.robot.bindings;

<<<<<<< HEAD:src/main/java/frc/robot/bindings/JoysticksBindings.java
import frc.robot.constants.Ports;
import frc.utils.controllers.joysticks.SmartJoystick;
=======
import frc.utils.joysticks.JoystickPorts;
import frc.utils.joysticks.SmartJoystick;
>>>>>>> origin/cr-java:src/main/java/frc/robot/JoysticksBindings.java

public class JoysticksBindings {

    private static final SmartJoystick MAIN_JOYSTICK = new SmartJoystick(JoystickPorts.MAIN);
    private static final SmartJoystick SECOND_JOYSTICK = new SmartJoystick(JoystickPorts.SECOND);
    private static final SmartJoystick THIRD_JOYSTICK = new SmartJoystick(JoystickPorts.THIRD);
    private static final SmartJoystick FOURTH_JOYSTICK = new SmartJoystick(JoystickPorts.FOURTH);
    private static final SmartJoystick FIFTH_JOYSTICK = new SmartJoystick(JoystickPorts.FIFTH);
    private static final SmartJoystick SIXTH_JOYSTICK = new SmartJoystick(JoystickPorts.SIXTH);

    public static void configureBindings() {
        mainJoystickButtons();
        secondJoystickButtons();
        thirdJoystickButtons();
        fourthJoystickButtons();
        fifthJoystickButtons();
        sixthJoystickButtons();
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

    private static void fifthJoystickButtons() {
        SmartJoystick usedJoystick = FIFTH_JOYSTICK;
        // bindings
    }

    private static void sixthJoystickButtons() {
        SmartJoystick usedJoystick = SIXTH_JOYSTICK;
        // bindings
    }

}
