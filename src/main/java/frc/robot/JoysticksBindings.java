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

        // Reset Angle to 0
        usedJoystick.Y.onTrue(new InstantCommand(() ->
                RobotContainer.POSE_ESTIMATOR.resetHeading(AllianceRotation2d.fromBlueAllianceRotation(new Rotation2d()))
        ));
        // Reset Pose to (5, 5, 0Deg)
        usedJoystick.B.onTrue(new InstantCommand(() ->
                RobotContainer.POSE_ESTIMATOR.resetPose(AlliancePose2d.fromBlueAlliancePose(new Pose2d(
                        5,
                        5,
                        new Rotation2d()
                )))
        ));

        // Swerve X Pose
        usedJoystick.A.whileTrue(SwerveCommands.getLockSwerveCommand());
        // Swerve Wheels to 90 Degrees
        usedJoystick.X.whileTrue(SwerveCommands.getPointWheelsCommand(Rotation2d.fromDegrees(90)));

        // Rotate to 180 Deg
        usedJoystick.POV_UP.whileTrue(SwerveCommands.getRotateToAngleCommand(
                AllianceRotation2d.fromBlueAllianceRotation(Rotation2d.fromDegrees(180))
        ));
        // Rotate to -17 Deg
        usedJoystick.POV_DOWN.whileTrue(SwerveCommands.getRotateToAngleCommand(
                AllianceRotation2d.fromBlueAllianceRotation(Rotation2d.fromDegrees(-17))
        ));

        // Rotate Around FRONT_LEFT to 180 Deg
        usedJoystick.POV_LEFT.whileTrue(SwerveCommands.getRotateToAngleCommand(
                AllianceRotation2d.fromBlueAllianceRotation(Rotation2d.fromDegrees(-17)), RotateAxis.FRONT_LEFT_MODULE
        ));
        // Rotate Around BACK_RIGHT to -17 Deg
        usedJoystick.POV_RIGHT.whileTrue(SwerveCommands.getRotateToAngleCommand(
                AllianceRotation2d.fromBlueAllianceRotation(Rotation2d.fromDegrees(180)), RotateAxis.BACK_RIGHT_MODULE
        ));

        //Self Relative Drive
        usedJoystick.L3.whileTrue(SwerveCommands.getSelfDriveCommand(
                () -> usedJoystick.getAxisValue(SmartJoystick.Axis.LEFT_Y),
                () -> usedJoystick.getAxisValue(SmartJoystick.Axis.LEFT_X),
                () -> usedJoystick.getAxisValue(SmartJoystick.Axis.RIGHT_X)
        ));
        // Drive and Aim Assist to Speaker
        usedJoystick.L1.whileTrue(SwerveCommands.getRotateToSpeaker(
                () -> usedJoystick.getAxisValue(SmartJoystick.Axis.LEFT_Y),
                () -> usedJoystick.getAxisValue(SmartJoystick.Axis.LEFT_X),
                () -> usedJoystick.getAxisValue(SmartJoystick.Axis.RIGHT_X)
        ));
        // Drive Slow
        //todo - not working in my joystick
        //        usedJoystick.R1.whileTrue(SwerveCommands.getOpenLoopFieldRelativeDriveCommandSlow(
        //                () -> usedJoystick.getAxisValue(SmartJoystick.Axis.LEFT_Y),
        //                () -> usedJoystick.getAxisValue(SmartJoystick.Axis.LEFT_X),
        //                () -> usedJoystick.getAxisValue(SmartJoystick.Axis.RIGHT_X)
        //        ));

        //Rotate Around Module:
        //FRONT RIGHT
        usedJoystick.R2.whileTrue(SwerveCommands.getDriveAroundWheelCommand(
                () -> usedJoystick.getAxisValue(SmartJoystick.Axis.LEFT_Y),
                () -> usedJoystick.getAxisValue(SmartJoystick.Axis.LEFT_X),
                () -> usedJoystick.getAxisValue(SmartJoystick.Axis.RIGHT_X),
                RotateAxis::getRightFarRotateAxis
        ));
        //FRONT LEFT
        usedJoystick.L2.whileTrue(SwerveCommands.getDriveAroundWheelCommand(
                () -> usedJoystick.getAxisValue(SmartJoystick.Axis.LEFT_Y),
                () -> usedJoystick.getAxisValue(SmartJoystick.Axis.LEFT_X),
                () -> usedJoystick.getAxisValue(SmartJoystick.Axis.RIGHT_X),
                RotateAxis::getLeftFarRotateAxis
        ));

        // Default Drive
        RobotContainer.SWERVE.setDefaultCommand(SwerveCommands.getOpenLoopFieldRelativeDriveCommand(
                () -> usedJoystick.getAxisValue(SmartJoystick.Axis.LEFT_Y),
                () -> usedJoystick.getAxisValue(SmartJoystick.Axis.LEFT_X),
                () -> usedJoystick.getAxisValue(SmartJoystick.Axis.RIGHT_X)
        ));

        // Move To Pose (4, 4, 17Deg)
        usedJoystick.BACK.whileTrue(SwerveCommands.getDriveToPoseCommand(
                () -> AlliancePose2d.fromBlueAlliancePose(new Pose2d(4, 4, Rotation2d.fromDegrees(17))),
                SwerveConstants.REAL_TIME_CONSTRAINTS
        ));
        // Move To Pose (5, 8, 90Deg)
        usedJoystick.START.whileTrue(SwerveCommands.getDriveToPoseCommand(
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
