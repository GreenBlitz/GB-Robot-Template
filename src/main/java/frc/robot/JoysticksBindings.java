package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.constants.Ports;
import frc.robot.subsystems.swerve.SwerveCommands;
import frc.robot.subsystems.swerve.SwerveConstants;
import frc.robot.subsystems.swerve.swervestatehelpers.AimAssist;
import frc.robot.subsystems.swerve.swervestatehelpers.RotateAxis;
import frc.utils.joysticks.SmartJoystick;
import frc.utils.mirrorutils.MirrorablePose2d;
import frc.utils.mirrorutils.MirrorableRotation2d;

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
        usedJoystick.Y.onTrue(new InstantCommand(() -> RobotContainer.POSE_ESTIMATOR.resetHeading(new Rotation2d())));
        // Reset Pose to (5, 5, 0Deg)
        usedJoystick.B.onTrue(new InstantCommand(() -> RobotContainer.POSE_ESTIMATOR.resetPose(
                new MirrorablePose2d(5, 5, new Rotation2d(), true).get()
        )));

        // Swerve X Pose
        usedJoystick.A.whileTrue(SwerveCommands.getLockSwerveCommand());
        // Swerve Wheels to 90 Degrees
        usedJoystick.X.whileTrue(SwerveCommands.getPointWheelsCommand(MirrorableRotation2d.fromDegrees(90, false)));

        // Rotate to 180 Deg
        usedJoystick.POV_UP.whileTrue(SwerveCommands.getRotateToAngleCommand(
                MirrorableRotation2d.fromDegrees(180, false)
        ));
        // Rotate to -17 Deg
        usedJoystick.POV_DOWN.whileTrue(SwerveCommands.getRotateToAngleCommand(
                MirrorableRotation2d.fromDegrees(-17, false)
        ));

        // Rotate Around FRONT_LEFT to 180 Deg
        usedJoystick.POV_LEFT.whileTrue(SwerveCommands.getRotateToAngleCommand(
                MirrorableRotation2d.fromDegrees(-17, false), RotateAxis.FRONT_LEFT_MODULE
        ));
        // Rotate Around BACK_RIGHT to -17 Deg
        usedJoystick.POV_RIGHT.whileTrue(SwerveCommands.getRotateToAngleCommand(
                MirrorableRotation2d.fromDegrees(180, false), RotateAxis.BACK_RIGHT_MODULE
        ));

        //Self Relative Drive
        usedJoystick.L3.whileTrue(SwerveCommands.getSelfDriveCommand(
                () -> usedJoystick.getSquaredSensitiveAxis(SmartJoystick.Axis.LEFT_Y),
                () -> usedJoystick.getSquaredSensitiveAxis(SmartJoystick.Axis.LEFT_X),
                () -> usedJoystick.getSensitiveJoystickValue(SmartJoystick.Axis.RIGHT_X)
        ));
        //Drive and Aim Assist to Speaker
        usedJoystick.L1.whileTrue(SwerveCommands.getDriveWithAimAssist(
                () -> usedJoystick.getSquaredSensitiveAxis(SmartJoystick.Axis.LEFT_Y),
                () -> usedJoystick.getSquaredSensitiveAxis(SmartJoystick.Axis.LEFT_X),
                () -> usedJoystick.getSensitiveJoystickValue(SmartJoystick.Axis.RIGHT_X),
                AimAssist.SPEAKER
        ));

        //Drive and Aim Assist to Speaker and Rotate around front Left
        //        usedJoystick.L1.whileTrue(SwerveCommands.debugCommand(
        //                () -> usedJoystick.getAxisValue(SmartJoystick.Axis.LEFT_Y),
        //                () -> usedJoystick.getAxisValue(SmartJoystick.Axis.LEFT_X),
        //                () -> usedJoystick.getAxisValue(SmartJoystick.Axis.RIGHT_X)
        //        ));

        //Drive Slow
        //        usedJoystick.Y.whileTrue(SwerveCommands.getOpenLoopFieldRelativeDriveCommandSlow(
        //                () -> usedJoystick.getAxisValue(SmartJoystick.Axis.LEFT_Y),
        //                () -> usedJoystick.getAxisValue(SmartJoystick.Axis.LEFT_X),
        //                () -> usedJoystick.getAxisValue(SmartJoystick.Axis.RIGHT_X)
        //        ));

        //Rotate Around Module:
        usedJoystick.R2.whileTrue(SwerveCommands.getDriveAroundWheelCommand(
                () -> usedJoystick.getSquaredSensitiveAxis(SmartJoystick.Axis.LEFT_Y),
                () -> usedJoystick.getSquaredSensitiveAxis(SmartJoystick.Axis.LEFT_X),
                () -> usedJoystick.getSensitiveJoystickValue(SmartJoystick.Axis.RIGHT_X),
                RotateAxis::getRightFarRotateAxis
        ));
        usedJoystick.L2.whileTrue(SwerveCommands.getDriveAroundWheelCommand(
                () -> usedJoystick.getSquaredSensitiveAxis(SmartJoystick.Axis.LEFT_Y),
                () -> usedJoystick.getSquaredSensitiveAxis(SmartJoystick.Axis.LEFT_X),
                () -> usedJoystick.getSensitiveJoystickValue(SmartJoystick.Axis.RIGHT_X),
                RotateAxis::getLeftFarRotateAxis
        ));

        // Default Drive
        RobotContainer.SWERVE.setDefaultCommand(SwerveCommands.getDriveCommand(
                () -> usedJoystick.getSquaredSensitiveAxis(SmartJoystick.Axis.LEFT_Y),
                () -> 0,//usedJoystick.getSquaredSensitiveAxis(SmartJoystick.Axis.LEFT_X),
                () -> usedJoystick.getSensitiveJoystickValue(SmartJoystick.Axis.RIGHT_X)
        ));

        // Move To Pose (4, 4, 17Deg)
        usedJoystick.BACK.whileTrue(SwerveCommands.getDriveToPoseCommand(
                () -> new MirrorablePose2d(4, 4, Rotation2d.fromDegrees(17), false),
                SwerveConstants.REAL_TIME_CONSTRAINTS
        ));
        // Move To Pose (5, 8, 90Deg)
        usedJoystick.START.whileTrue(SwerveCommands.getDriveToPoseCommand(
                () -> new MirrorablePose2d(5, 8, Rotation2d.fromDegrees(90), true),
                SwerveConstants.REAL_TIME_CONSTRAINTS
        ));
    }

    private static void secondJoystickButtons() {
        SmartJoystick usedJoystick = SECOND_JOYSTICK;
        // bindings

        usedJoystick.A.whileTrue(SwerveCommands.getWheelRadiusCalibrationCommand());
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
