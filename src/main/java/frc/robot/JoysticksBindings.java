package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.constants.Ports;
import frc.robot.subsystems.swerve.SwerveCommands;
import frc.robot.subsystems.swerve.SwerveConstants;
import frc.robot.subsystems.swerve.swervestatehelpers.RotateAxis;
import frc.utils.allianceutils.AlliancePose2d;
import frc.utils.allianceutils.AllianceRotation2d;
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
        // - lock x - CHECKED
        // - point wheels - CHECKED
        // - slow mode - CHECKED
        // - follow path - CHECKED
        // - closed loop drive - Doing Noises, Not Fun
        // - rotate to angle - CHECKED
        // - drive around wheel - CHECKED
        // - rotate to angle around wheel - need to check on carpet
        // - self relative drive - CHECKED
        // - pose estimator resets - CHECKED

        //reset angle pose estim
        usedJoystick.R1.onTrue(new InstantCommand(() ->
                RobotContainer.POSE_ESTIMATOR.resetHeading(AllianceRotation2d.fromBlueAllianceRotation(new Rotation2d()))
        ));
        usedJoystick.L1.onTrue(new InstantCommand(() ->
                RobotContainer.POSE_ESTIMATOR.resetPose(AlliancePose2d.fromBlueAlliancePose(new Pose2d(
                        5,
                        5,
                        new Rotation2d()
                )))
        ));

        usedJoystick.POV_UP.whileTrue(SwerveCommands.getRotateToAngleCommand(
                AllianceRotation2d.fromBlueAllianceRotation(Rotation2d.fromDegrees(180))
        ));
        usedJoystick.POV_DOWN.whileTrue(SwerveCommands.getRotateToAngleCommand(
                AllianceRotation2d.fromBlueAllianceRotation(Rotation2d.fromDegrees(-17))
        ));
        usedJoystick.START.whileTrue(SwerveCommands.getRotateToAngleCommand(
                AllianceRotation2d.fromBlueAllianceRotation(Rotation2d.fromDegrees(-17)), RotateAxis.FRONT_LEFT_MODULE
        ));
        usedJoystick.BACK.whileTrue(SwerveCommands.getRotateToAngleCommand(
                AllianceRotation2d.fromBlueAllianceRotation(Rotation2d.fromDegrees(180)), RotateAxis.BACK_RIGHT_MODULE
        ));

        usedJoystick.B.whileTrue(SwerveCommands.getDriveAroundWheelCommand(
                () -> usedJoystick.getAxisValue(SmartJoystick.Axis.LEFT_Y),
                () -> usedJoystick.getAxisValue(SmartJoystick.Axis.LEFT_X),
                () -> usedJoystick.getAxisValue(SmartJoystick.Axis.RIGHT_X),
                RotateAxis.BACK_RIGHT_MODULE
        ));
        usedJoystick.Y.whileTrue(SwerveCommands.getSelfDriveCommand(
                () -> usedJoystick.getAxisValue(SmartJoystick.Axis.LEFT_Y),
                () -> usedJoystick.getAxisValue(SmartJoystick.Axis.LEFT_X),
                () -> usedJoystick.getAxisValue(SmartJoystick.Axis.RIGHT_X)
        ));
        usedJoystick.L2.whileTrue(SwerveCommands.getRotateToSpeaker(
                () -> usedJoystick.getAxisValue(SmartJoystick.Axis.LEFT_Y),
                () -> usedJoystick.getAxisValue(SmartJoystick.Axis.LEFT_X),
                () -> usedJoystick.getAxisValue(SmartJoystick.Axis.RIGHT_X)
        ));
        usedJoystick.R2.whileTrue(SwerveCommands.getOpenLoopFieldRelativeDriveCommandSlow(
                () -> usedJoystick.getAxisValue(SmartJoystick.Axis.LEFT_Y),
                () -> usedJoystick.getAxisValue(SmartJoystick.Axis.LEFT_X),
                () -> usedJoystick.getAxisValue(SmartJoystick.Axis.RIGHT_X)
        ));
        RobotContainer.SWERVE.setDefaultCommand(SwerveCommands.getOpenLoopFieldRelativeDriveCommand(
                () -> usedJoystick.getAxisValue(SmartJoystick.Axis.LEFT_Y),
                () -> usedJoystick.getAxisValue(SmartJoystick.Axis.LEFT_X),
                () -> usedJoystick.getAxisValue(SmartJoystick.Axis.RIGHT_X)
        ));

        usedJoystick.A.whileTrue(SwerveCommands.getLockSwerveCommand());
        usedJoystick.X.whileTrue(SwerveCommands.getPointWheelsCommand(Rotation2d.fromDegrees(90)));

        usedJoystick.POV_LEFT.whileTrue(SwerveCommands.getDriveToPoseCommand(
                () -> AlliancePose2d.fromBlueAlliancePose(new Pose2d(4, 4, Rotation2d.fromDegrees(17))),
                SwerveConstants.REAL_TIME_CONSTRAINTS
        ));
        usedJoystick.POV_RIGHT.whileTrue(SwerveCommands.getDriveToPoseCommand(
                () -> AlliancePose2d.fromBlueAlliancePose(new Pose2d(5, 8, Rotation2d.fromDegrees(90))),
                SwerveConstants.REAL_TIME_CONSTRAINTS
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
