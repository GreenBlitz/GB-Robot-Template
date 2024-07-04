package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.subsystems.swerve.SwerveCommands;
import frc.robot.subsystems.swerve.swervestatehelpers.AimAssist;
import frc.robot.subsystems.swerve.swervestatehelpers.RotateAxis;
import frc.utils.joysticks.Axis;
import frc.utils.joysticks.JoystickPorts;
import frc.utils.joysticks.SmartJoystick;


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
        usedJoystick.Y.onTrue(new InstantCommand(() -> Robot.resetHeading(new Rotation2d())));
        // Reset Pose to (5, 5, 0Deg)
        usedJoystick.B.onTrue(new InstantCommand(() -> Robot.resetPose(new Pose2d(5, 5, new Rotation2d()))));

        // Swerve X Pose
        usedJoystick.A.whileTrue(SwerveCommands.pointWheelsInX());
        // Swerve Wheels to 90 Degrees
        usedJoystick.X.whileTrue(SwerveCommands.pointWheels(Rotation2d.fromDegrees(90), true));

        // Rotate to 180 Deg
        usedJoystick.POV_UP.whileTrue(SwerveCommands.rotateToAngle(Rotation2d.fromDegrees(180)));
        // Rotate to -17 Deg
        usedJoystick.POV_DOWN.whileTrue(SwerveCommands.rotateToAngle(Rotation2d.fromDegrees(-17)));

        // Rotate Around FRONT_LEFT to 180 Deg
        usedJoystick.POV_LEFT.whileTrue(SwerveCommands.rotateToAngle(Rotation2d.fromDegrees(-17), RotateAxis.FRONT_LEFT_MODULE));
        // Rotate Around BACK_RIGHT to -17 Deg
        usedJoystick.POV_RIGHT.whileTrue(SwerveCommands.rotateToAngle(Rotation2d.fromDegrees(180), RotateAxis.BACK_RIGHT_MODULE));

        //Self Relative Drive
        usedJoystick.L3.whileTrue(SwerveCommands.driveSelfRelative(
                () -> usedJoystick.getAxisValue(Axis.LEFT_Y),
                () -> usedJoystick.getAxisValue(Axis.LEFT_X),
                () -> usedJoystick.getSensitiveAxisValue(Axis.RIGHT_X)
        ));
        //Drive and Aim Assist to Speaker
        usedJoystick.L1.whileTrue(SwerveCommands.driveWithAimAssist(
                () -> usedJoystick.getAxisValue(Axis.LEFT_Y),
                () -> usedJoystick.getAxisValue(Axis.LEFT_X),
                () -> usedJoystick.getSensitiveAxisValue(Axis.RIGHT_X),
                AimAssist.SPEAKER
        ));

        //Drive and Aim Assist to Speaker and Rotate around front Left
        //        usedJoystick.L1.whileTrue(SwerveCommands.debugCommand(
        //                () -> usedJoystick.getAxisValue(Axis.LEFT_Y),
        //                () -> usedJoystick.getAxisValue(Axis.LEFT_X),
        //                () -> usedJoystick.getAxisValue(Axis.RIGHT_X)
        //        ));

        //Drive Slow
        //        usedJoystick.Y.whileTrue(SwerveCommands.getOpenLoopFieldRelativeDriveCommandSlow(
        //                () -> usedJoystick.getAxisValue(Axis.LEFT_Y),
        //                () -> usedJoystick.getAxisValue(Axis.LEFT_X),
        //                () -> usedJoystick.getAxisValue(Axis.RIGHT_X)
        //        ));

        //Rotate Around Module:
        usedJoystick.getAxisAsButton(Axis.RIGHT_TRIGGER).whileTrue(SwerveCommands.driveAroundWheel(
                () -> usedJoystick.getAxisValue(Axis.LEFT_Y),
                () -> usedJoystick.getAxisValue(Axis.LEFT_X),
                () -> usedJoystick.getSensitiveAxisValue(Axis.RIGHT_X),
                RotateAxis::getRightFarRotateAxis
        ));
        usedJoystick.getAxisAsButton(Axis.LEFT_TRIGGER).whileTrue(SwerveCommands.driveAroundWheel(
                () -> usedJoystick.getAxisValue(Axis.LEFT_Y),
                () -> usedJoystick.getAxisValue(Axis.LEFT_X),
                () -> usedJoystick.getSensitiveAxisValue(Axis.RIGHT_X),
                RotateAxis::getLeftFarRotateAxis
        ));

        // Default Drive
        Robot.swerve.setDefaultCommand(SwerveCommands.drive(
                () -> usedJoystick.getAxisValue(Axis.LEFT_Y),
                () -> usedJoystick.getAxisValue(Axis.LEFT_X),
                () -> usedJoystick.getSensitiveAxisValue(Axis.RIGHT_X)
        ));

        // Move To Pose (4, 4, 17Deg)
        usedJoystick.BACK.whileTrue(SwerveCommands.driveToPose(new Pose2d(4, 4, Rotation2d.fromDegrees(17))));
        // Move To Pose (5, 8, 90Deg)
        usedJoystick.START.whileTrue(SwerveCommands.driveToPose(new Pose2d(6, 6, Rotation2d.fromDegrees(90))));
    }

    private static void secondJoystickButtons() {
        SmartJoystick usedJoystick = SECOND_JOYSTICK;
        // bindings

        usedJoystick.A.whileTrue(SwerveCommands.wheelRadiusCalibration());
        usedJoystick.B.whileTrue(SwerveCommands.steerCalibration(true, SysIdRoutine.Direction.kForward));
        usedJoystick.Y.whileTrue(SwerveCommands.driveCalibration(true, SysIdRoutine.Direction.kForward));
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
