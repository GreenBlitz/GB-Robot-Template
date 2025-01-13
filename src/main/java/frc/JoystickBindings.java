package frc;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.joysticks.Axis;
import frc.joysticks.BindSet;
import frc.robot.Robot;
import frc.joysticks.SmartJoystick;
import frc.robot.subsystems.swerve.states.SwerveState;

public class JoystickBindings {

	public static void configureBindings(SmartJoystick joystick, Robot robot) {
		swerveTest(joystick, robot);
		swerveCalibration(joystick, robot);
	}

	private static void swerveTest(SmartJoystick joystick, Robot robot) {
		bindSetTrigger(joystick, joystick.Y, BindSet.SWERVE_TEST)
			.onTrue(new InstantCommand(() -> robot.getPoseEstimator().resetHeading(new Rotation2d())));
		bindSetTrigger(joystick, joystick.B, BindSet.SWERVE_TEST)
			.onTrue(new InstantCommand(() -> robot.getPoseEstimator().resetPose(new Pose2d(5, 5, new Rotation2d()))));

		bindSetTrigger(joystick, joystick.A, BindSet.SWERVE_TEST).whileTrue(robot.getSwerve().getCommandsBuilder().pointWheelsInX());
		bindSetTrigger(joystick, joystick.X, BindSet.SWERVE_TEST)
			.whileTrue(robot.getSwerve().getCommandsBuilder().pointWheels(Rotation2d.fromDegrees(90), true));

		bindSetTrigger(joystick, joystick.POV_UP, BindSet.SWERVE_TEST)
			.whileTrue(robot.getSwerve().getCommandsBuilder().turnToHeading(Rotation2d.fromDegrees(180)));
		bindSetTrigger(joystick, joystick.POV_DOWN, BindSet.SWERVE_TEST).whileTrue(
			robot.getSwerve()
				.getCommandsBuilder()
				.turnToHeading(Rotation2d.fromDegrees(-17))
				.until(
					() -> robot.getSwerve()
						.isAtHeading(Rotation2d.fromDegrees(-17), Tolerances.SWERVE_HEADING, Tolerances.ROTATION_VELOCITY_DEADBAND)
				)
		);

		bindSetTrigger(joystick, joystick.POV_LEFT, BindSet.SWERVE_TEST)
			.whileTrue(robot.getSwerve().getCommandsBuilder().turnToHeading(Rotation2d.fromDegrees(-17), RotateAxis.FRONT_LEFT_MODULE));
		bindSetTrigger(joystick, joystick.POV_RIGHT, BindSet.SWERVE_TEST)
			.whileTrue(robot.getSwerve().getCommandsBuilder().turnToHeading(Rotation2d.fromDegrees(180), RotateAxis.BACK_RIGHT_MODULE));

		bindSetTrigger(joystick, joystick.L3, BindSet.SWERVE_TEST).whileTrue(
			robot.getSwerve()
				.getCommandsBuilder()
				.driveByState(
					() -> joystick.getAxisValue(Axis.LEFT_Y),
					() -> joystick.getAxisValue(Axis.LEFT_X),
					() -> joystick.getSensitiveAxisValue(Axis.RIGHT_X),
					SwerveState.DEFAULT_DRIVE.withAimAssist(AimAssist.AMP)
				)
		);
		bindSetTrigger(joystick, joystick.L1, BindSet.SWERVE_TEST).whileTrue(
			robot.getSwerve()
				.getCommandsBuilder()
				.driveByState(
					() -> joystick.getAxisValue(Axis.LEFT_Y),
					() -> joystick.getAxisValue(Axis.LEFT_X),
					() -> joystick.getSensitiveAxisValue(Axis.RIGHT_X),
					SwerveState.DEFAULT_DRIVE.withDriveRelative(DriveRelative.ROBOT_RELATIVE).withAimAssist(AimAssist.NOTE)
				)
		);
		bindSetTrigger(joystick, joystick.R1, BindSet.SWERVE_TEST).whileTrue(
			robot.getSwerve()
				.getCommandsBuilder()
				.driveByState(
					() -> joystick.getAxisValue(Axis.LEFT_Y),
					() -> joystick.getAxisValue(Axis.LEFT_X),
					() -> joystick.getSensitiveAxisValue(Axis.RIGHT_X),
					SwerveState.DEFAULT_DRIVE.withAimAssist(AimAssist.SPEAKER)
				)
		);

		bindSetTrigger(joystick, joystick.getAxisAsButton(Axis.RIGHT_TRIGGER), BindSet.SWERVE_TEST).whileTrue(
			robot.getSwerve()
				.getCommandsBuilder()
				.driveByState(
					() -> joystick.getAxisValue(Axis.LEFT_Y),
					() -> joystick.getAxisValue(Axis.LEFT_X),
					() -> joystick.getSensitiveAxisValue(Axis.RIGHT_X),
					() -> SwerveState.DEFAULT_DRIVE.withRotateAxis(robot.getSwerve().getStateHandler().getFarRightRotateAxis())
				)
		);
		bindSetTrigger(joystick, joystick.getAxisAsButton(Axis.LEFT_TRIGGER), BindSet.SWERVE_TEST).whileTrue(
			robot.getSwerve()
				.getCommandsBuilder()
				.driveByState(
					() -> joystick.getAxisValue(Axis.LEFT_Y),
					() -> joystick.getAxisValue(Axis.LEFT_X),
					() -> joystick.getSensitiveAxisValue(Axis.RIGHT_X),
					() -> SwerveState.DEFAULT_DRIVE.withRotateAxis(robot.getSwerve().getStateHandler().getFarLeftRotateAxis())
				)
		);

		bindSetTrigger(joystick, BindSet.SWERVE_TEST).whileTrue(
			robot.getSwerve()
				.getCommandsBuilder()
				.drive(
					() -> joystick.getAxisValue(Axis.LEFT_Y),
					() -> joystick.getAxisValue(Axis.LEFT_X),
					() -> joystick.getSensitiveAxisValue(Axis.RIGHT_X)
				)
		);

		bindSetTrigger(joystick, joystick.BACK, BindSet.SWERVE_TEST).whileTrue(
			robot.getSwerve()
				.getCommandsBuilder()
				.driveToPose(robot.getPoseEstimator()::getCurrentPose, () -> new Pose2d(4, 4, Rotation2d.fromDegrees(17)))
				.until(() -> robot.getSuperStructure().isAtPose(new Pose2d(4, 4, Rotation2d.fromDegrees(17))))
		);
		bindSetTrigger(joystick, joystick.START, BindSet.SWERVE_TEST).whileTrue(
			robot.getSwerve()
				.getCommandsBuilder()
				.driveToPose(robot.getPoseEstimator()::getCurrentPose, () -> new Pose2d(6, 6, Rotation2d.fromDegrees(90)))
				.until(() -> robot.getSuperStructure().isAtPose(new Pose2d(6, 6, Rotation2d.fromDegrees(90))))
		);
	}

	private static void swerveCalibration(SmartJoystick joystick, Robot robot) {
		bindSetTrigger(joystick, joystick.A, BindSet.SWERVE_CALIBRATION)
			.whileTrue(robot.getSwerve().getCommandsBuilder().wheelRadiusCalibration());
		bindSetTrigger(joystick, joystick.B, BindSet.SWERVE_CALIBRATION)
			.whileTrue(robot.getSwerve().getCommandsBuilder().steerCalibration(true, SysIdRoutine.Direction.kForward));
		bindSetTrigger(joystick, joystick.Y, BindSet.SWERVE_CALIBRATION)
			.whileTrue(robot.getSwerve().getCommandsBuilder().driveCalibration(true, SysIdRoutine.Direction.kForward));

		bindSetTrigger(joystick, joystick.POV_DOWN, BindSet.SWERVE_CALIBRATION)
			.whileTrue(robot.getSwerve().getCommandsBuilder().drive(() -> 0.2, () -> 0, () -> 0));
		bindSetTrigger(joystick, joystick.POV_LEFT, BindSet.SWERVE_CALIBRATION)
			.whileTrue(robot.getSwerve().getCommandsBuilder().drive(() -> 0.5, () -> 0, () -> 0));
		bindSetTrigger(joystick, joystick.POV_RIGHT, BindSet.SWERVE_CALIBRATION)
			.whileTrue(robot.getSwerve().getCommandsBuilder().drive(() -> -0.2, () -> 0, () -> 0));
		bindSetTrigger(joystick, joystick.POV_UP, BindSet.SWERVE_CALIBRATION)
			.whileTrue(robot.getSwerve().getCommandsBuilder().drive(() -> -0.5, () -> 0, () -> 0));
	}

	private static Trigger bindSetTrigger(SmartJoystick joystick, Trigger bind, BindSet bindSetRequirement) {
		return bind.and(bindSetTrigger(joystick, bindSetRequirement));
	}

	private static Trigger bindSetTrigger(SmartJoystick joystick, BindSet bindSetRequirement) {
		return new Trigger(() -> bindSetRequirement == joystick.getBindSet());
	}


}
