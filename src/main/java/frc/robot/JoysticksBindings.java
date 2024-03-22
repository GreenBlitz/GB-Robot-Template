package frc.robot;

import frc.robot.constants.Ports;
import frc.utils.humaninterfacedevices.SmartJoystick;

public class JoysticksBindings {

    private final SmartJoystick MAIN_JOYSTICK = new SmartJoystick(Ports.Joystick.MAIN);
    private final SmartJoystick SECOND_JOYSTICK = new SmartJoystick(Ports.Joystick.SECOND);
    private final SmartJoystick THIRD_JOYSTICK = new SmartJoystick(Ports.Joystick.THIRD);
    private final SmartJoystick FOURTH_JOYSTICK = new SmartJoystick(Ports.Joystick.FOURTH);

    public static void configureBindings(){

    }

    public void mainJoystickButtons(){
        SmartJoystick usedJoystick = MAIN_JOYSTICK;
        //bindings
    }

    public void secondJoystickButtons(){
        SmartJoystick usedJoystick = SECOND_JOYSTICK;
        //bindings
    }

    public void thirdJoystickButtons(){
        SmartJoystick usedJoystick = THIRD_JOYSTICK;
        //bindings
    }

    public void fourthJoystickButtons(){
        SmartJoystick usedJoystick = FOURTH_JOYSTICK;
        //bindings
    }



}
