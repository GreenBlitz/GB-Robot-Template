package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.subsystems.swerve.SwerveState;
import frc.robot.subsystems.swerve.swervestatehelpers.AimAssist;
import frc.robot.subsystems.swerve.swervestatehelpers.DriveRelative;
import frc.robot.subsystems.swerve.swervestatehelpers.LoopMode;
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

	public static void configureBindings(Robot robot) {
		mainJoystickButtons(robot);
		secondJoystickButtons(robot);
		thirdJoystickButtons(robot);
		fourthJoystickButtons(robot);
		fifthJoystickButtons(robot);
		sixthJoystickButtons(robot);
	}

	private static void mainJoystickButtons(Robot robot) {
		SmartJoystick usedJoystick = MAIN_JOYSTICK;
		// bindings...

		usedJoystick.Y.onTrue(new InstantCommand(() -> robot.getPoseEstimator().resetHeading(new Rotation2d())));
		usedJoystick.B.onTrue(new InstantCommand(() -> robot.getPoseEstimator().resetPose(new Pose2d(5, 5, new Rotation2d()))));

		usedJoystick.A.whileTrue(robot.getSwerve().getCommandsBuilder().pointWheelsInX());
		usedJoystick.X.whileTrue(robot.getSwerve().getCommandsBuilder().pointWheels(Rotation2d.fromDegrees(90), true));

		usedJoystick.POV_UP.whileTrue(robot.getSwerve().getCommandsBuilder().rotateToAngle(Rotation2d.fromDegrees(180)));
		usedJoystick.POV_DOWN.whileTrue(robot.getSwerve().getCommandsBuilder().rotateToAngle(Rotation2d.fromDegrees(-17)));

		usedJoystick.POV_LEFT.whileTrue(
			robot.getSwerve().getCommandsBuilder().rotateToAngle(Rotation2d.fromDegrees(-17), RotateAxis.FRONT_LEFT_MODULE)
		);
		usedJoystick.POV_RIGHT.whileTrue(
			robot.getSwerve().getCommandsBuilder().rotateToAngle(Rotation2d.fromDegrees(180), RotateAxis.BACK_RIGHT_MODULE)
		);

		usedJoystick.L3.whileTrue(
			robot.getSwerve()
				.getCommandsBuilder()
				.driveState(
					() -> usedJoystick.getAxisValue(Axis.LEFT_Y),
					() -> usedJoystick.getAxisValue(Axis.LEFT_X),
					() -> usedJoystick.getSensitiveAxisValue(Axis.RIGHT_X),
					SwerveState.DEFAULT_DRIVE.withDriveRelative(DriveRelative.ROBOT_RELATIVE)
				)
		);
		usedJoystick.L1.whileTrue(
			robot.getSwerve()
				.getCommandsBuilder()
				.driveState(
					() -> usedJoystick.getAxisValue(Axis.LEFT_Y),
					() -> usedJoystick.getAxisValue(Axis.LEFT_X),
					() -> usedJoystick.getSensitiveAxisValue(Axis.RIGHT_X),
					SwerveState.DEFAULT_DRIVE.withAimAssist(AimAssist.SPEAKER)
				)
		);
		usedJoystick.R1.whileTrue(
			robot.getSwerve()
				.getCommandsBuilder()
				.driveState(
					() -> usedJoystick.getAxisValue(Axis.LEFT_Y),
					() -> usedJoystick.getAxisValue(Axis.LEFT_X),
					() -> usedJoystick.getSensitiveAxisValue(Axis.RIGHT_X),
					SwerveState.DEFAULT_DRIVE.withLoopMode(LoopMode.OPEN)
				)
		);

		usedJoystick.getAxisAsButton(Axis.RIGHT_TRIGGER)
			.whileTrue(
				robot.getSwerve()
					.getCommandsBuilder()
					.driveState(
						() -> usedJoystick.getAxisValue(Axis.LEFT_Y),
						() -> usedJoystick.getAxisValue(Axis.LEFT_X),
						() -> usedJoystick.getSensitiveAxisValue(Axis.RIGHT_X),
						() -> SwerveState.DEFAULT_DRIVE.withRotateAxis(RotateAxis.getRightFarRotateAxis())
					)
			);
		usedJoystick.getAxisAsButton(Axis.LEFT_TRIGGER)
			.whileTrue(
				robot.getSwerve()
					.getCommandsBuilder()
					.driveState(
						() -> usedJoystick.getAxisValue(Axis.LEFT_Y),
						() -> usedJoystick.getAxisValue(Axis.LEFT_X),
						() -> usedJoystick.getSensitiveAxisValue(Axis.RIGHT_X),
						() -> SwerveState.DEFAULT_DRIVE.withRotateAxis(RotateAxis.getLeftFarRotateAxis())
					)
			);

		robot.getSwerve()
			.setDefaultCommand(
				robot.getSwerve()
					.getCommandsBuilder()
					.drive(
						() -> usedJoystick.getAxisValue(Axis.LEFT_Y),
						() -> usedJoystick.getAxisValue(Axis.LEFT_X),
						() -> usedJoystick.getSensitiveAxisValue(Axis.RIGHT_X)
					)
			);

		usedJoystick.BACK.whileTrue(
			robot.getSwerve()
				.getCommandsBuilder()
				.driveToPose(
					robot.getPoseEstimator()::getCurrentPose,
					() -> new Pose2d(4, 4, Rotation2d.fromDegrees(17)),
					robot.getPoseEstimator()::isAtPose
				)
		);
		usedJoystick.START.whileTrue(
			robot.getSwerve()
				.getCommandsBuilder()
				.driveToPose(
					robot.getPoseEstimator()::getCurrentPose,
					() -> new Pose2d(6, 6, Rotation2d.fromDegrees(90)),
					robot.getPoseEstimator()::isAtPose
				)
		);
	}

	private static void secondJoystickButtons(Robot robot) {
		SmartJoystick usedJoystick = SECOND_JOYSTICK;
		// bindings...

		usedJoystick.A.whileTrue(robot.getSwerve().getCommandsBuilder().wheelRadiusCalibration());
		usedJoystick.B.whileTrue(robot.getSwerve().getCommandsBuilder().steerCalibration(true, SysIdRoutine.Direction.kForward));
		usedJoystick.Y.whileTrue(robot.getSwerve().getCommandsBuilder().driveCalibration(true, SysIdRoutine.Direction.kForward));
	}

	private static void thirdJoystickButtons(Robot robot) {
		SmartJoystick usedJoystick = THIRD_JOYSTICK;
		// bindings...
		usedJoystick.A.whileTrue(
			robot.getSwerve()
				.getCommandsBuilder()
				.driveToPose(
					robot.getPoseEstimator()::getCurrentPose,
					() -> new Pose2d(1, 8, Rotation2d.fromDegrees(90)),
					robot.getPoseEstimator()::isAtPose

				)
		);
		usedJoystick.X.whileTrue(
			robot.getSwerve()
				.getCommandsBuilder()
				.driveToPose(
					robot.getPoseEstimator()::getCurrentPose,
					() -> new Pose2d(6, 6, Rotation2d.fromDegrees(117)),
					robot.getPoseEstimator()::isAtPose
				)
		);
		usedJoystick.Y.whileTrue(
			robot.getSwerve()
				.getCommandsBuilder()
				.driveToPose(
					robot.getPoseEstimator()::getCurrentPose,
					() -> new Pose2d(7, 2, Rotation2d.fromDegrees(180)),
					robot.getPoseEstimator()::isAtPose
				)
		);
		usedJoystick.B.whileTrue(
			robot.getSwerve()
				.getCommandsBuilder()
				.driveToPose(
					robot.getPoseEstimator()::getCurrentPose,
					() -> new Pose2d(16, 6, Rotation2d.fromDegrees(-75)),
					robot.getPoseEstimator()::isAtPose
				)
		);
		usedJoystick.START.whileTrue(
			robot.getSwerve()
				.getCommandsBuilder()
				.driveToPose(
					robot.getPoseEstimator()::getCurrentPose,
					() -> new Pose2d(12, 8, Rotation2d.fromDegrees(14)),
					robot.getPoseEstimator()::isAtPose
				)
		);
		usedJoystick.BACK.whileTrue(
			robot.getSwerve()
				.getCommandsBuilder()
				.driveToPose(
					robot.getPoseEstimator()::getCurrentPose,
					() -> new Pose2d(10, 4, Rotation2d.fromDegrees(140)),
					robot.getPoseEstimator()::isAtPose

				)
		);
	}

	private static void fourthJoystickButtons(Robot robot) {
		SmartJoystick usedJoystick = FOURTH_JOYSTICK;
		// bindings...
		usedJoystick.A.whileTrue(robot.getSwerve().getCommandsBuilder().drive(() -> 0.2, () -> 0, () -> 0));
		usedJoystick.B.whileTrue(robot.getSwerve().getCommandsBuilder().drive(() -> 0.5, () -> 0, () -> 0));
		usedJoystick.X.whileTrue(robot.getSwerve().getCommandsBuilder().drive(() -> -0.2, () -> 0, () -> 0));
		usedJoystick.Y.whileTrue(robot.getSwerve().getCommandsBuilder().drive(() -> -0.5, () -> 0, () -> 0));
	}

	private static void fifthJoystickButtons(Robot robot) {
		SmartJoystick usedJoystick = FIFTH_JOYSTICK;
		// bindings...
	}

	private static void sixthJoystickButtons(Robot robot) {
		SmartJoystick usedJoystick = SIXTH_JOYSTICK;
		// bindings...
	}

}
