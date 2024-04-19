package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.constants.Ports;
import frc.utils.commands.InitExecuteCommand;
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
        usedJoystick.A.whileTrue(new InitExecuteCommand(
                () -> RobotContainer.SYS.setPositionControl(Rotation2d.fromRotations(1000)),
                System.out::println,
                RobotContainer.SYS
        ));
        usedJoystick.B.whileTrue(new InitExecuteCommand(
                () -> RobotContainer.SYS.setPositionControl(Rotation2d.fromRotations(-100)),
                System.out::println,
                RobotContainer.SYS
        ));
        usedJoystick.X.whileTrue(new InitExecuteCommand(
                () -> RobotContainer.SYS.setVelocityControl(10),
                System.out::println,
                RobotContainer.SYS
        ));
        usedJoystick.Y.whileTrue(new InitExecuteCommand(
                () -> RobotContainer.SYS.setVelocityControl(50),
                System.out::println,
                RobotContainer.SYS
        ));
        usedJoystick.START.whileTrue(new InitExecuteCommand(
                RobotContainer.SYS::stop,
                RobotContainer.SYS::stop,
                RobotContainer.SYS
        ));
        usedJoystick.POV_UP.whileTrue(new InitExecuteCommand(
                () -> RobotContainer.SYS.setVoltageControl(6),
                System.out::println,
                RobotContainer.SYS
        ));
        usedJoystick.POV_DOWN.whileTrue(new InitExecuteCommand(
                () -> RobotContainer.SYS.setVoltageControl(-6),
                System.out::println,
                RobotContainer.SYS
        ));
        Command defaultCommand = new InstantCommand(RobotContainer.SYS::stop);
        defaultCommand.addRequirements(RobotContainer.SYS);
        RobotContainer.SYS.setDefaultCommand(defaultCommand);
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
