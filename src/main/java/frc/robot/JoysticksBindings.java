package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.constants.Ports;
import frc.robot.subsystems.swerve.SwerveCommands;
import frc.utils.joysticks.SmartJoystick;

public class JoysticksBindings {

    private static final SmartJoystick MAIN_JOYSTICK = new SmartJoystick(Ports.JoystickDriverStationPorts.MAIN);

    private static final SmartJoystick SECOND_JOYSTICK = new SmartJoystick(Ports.JoystickDriverStationPorts.SECOND);

    private static final SmartJoystick THIRD_JOYSTICK = new SmartJoystick(Ports.JoystickDriverStationPorts.THIRD);

    private static final SmartJoystick FOURTH_JOYSTICK = new SmartJoystick(Ports.JoystickDriverStationPorts.FOURTH);

    public static void configureBindings() {
        mainJoystickButtons();
        secondJoystickButtons();
        thirdJoystickButtons();
        fourthJoystickButtons();
    }

    private static void mainJoystickButtons() {
        SmartJoystick usedJoystick = MAIN_JOYSTICK;
        // bindings

        //todo
        // - lock x
        // - point wheels
        // - slow mode - CHECKED
        // - follow path
        // - closed loop drive
        // - rotate to angle
        // - drive around wheel
        // - rotate to angle around wheel
        // - self relative drive

        usedJoystick.A.whileTrue(SwerveCommands.getLockSwerveCommand());
        usedJoystick.X.whileTrue(SwerveCommands.getPointWheelsCommand(Rotation2d.fromDegrees(90)));
        usedJoystick.L2.whileTrue(SwerveCommands.getOpenLoopFieldRelativeDriveCommandSlow(
                () -> usedJoystick.getAxisValue(SmartJoystick.Axis.LEFT_Y),
                () -> usedJoystick.getAxisValue(SmartJoystick.Axis.LEFT_X),
                () -> usedJoystick.getAxisValue(SmartJoystick.Axis.RIGHT_X)
        ));

        RobotContainer.SWERVE.setDefaultCommand(SwerveCommands.getOpenLoopFieldRelativeDriveCommand(
                () -> usedJoystick.getAxisValue(SmartJoystick.Axis.LEFT_Y),
                () -> usedJoystick.getAxisValue(SmartJoystick.Axis.LEFT_X),
                () -> usedJoystick.getAxisValue(SmartJoystick.Axis.RIGHT_X)
        ));
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

}
