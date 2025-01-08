package frc;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.joysticks.Axis;
import frc.joysticks.JoystickPorts;
import frc.joysticks.SmartJoystick;
import frc.robot.Robot;
import frc.robot.structures.Tolerances;
import frc.robot.subsystems.swerve.SwerveMath;
import frc.robot.subsystems.swerve.states.SwerveState;
import frc.robot.subsystems.swerve.states.aimassist.AimAssist;
import frc.robot.subsystems.swerve.states.DriveRelative;
import frc.robot.subsystems.swerve.states.RotateAxis;
import org.littletonrobotics.junction.Logger;

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
		usedJoystick.Y.onTrue(new InstantCommand(() -> robot.getAprilTagVisionSources().switchBotPoses()));
//		usedJoystick.B.onTrue(new InstantCommand(() -> robot.getSwerve().getCommandsBuilder());

		usedJoystick.A.whileTrue(robot.getSwerve().getCommandsBuilder().driveToPose(
			() -> robot.getPoseEstimator().getEstimatedPose(),
			() -> new Pose2d(13.970, 4.344, Rotation2d.fromRadians(-3.125))
		));

		double y = 4.3;
		usedJoystick.B.onTrue(new InstantCommand(() -> robot.getHeadingEstimator().reset(Rotation2d.fromDegrees(180))));
		usedJoystick.X.whileTrue(
			(
				new SequentialCommandGroup(
					robot.getSwerve().getCommandsBuilder().driveToPose(
						() -> robot.getPoseEstimator().getEstimatedPose(),
						() -> new Pose2d(new Translation2d(15, y), Rotation2d.fromDegrees(180))
					),
					robot.getSwerve().getCommandsBuilder().driveToPose(
						() -> robot.getPoseEstimator().getEstimatedPose(),
						() -> new Pose2d(new Translation2d(14.05, /*4.3*/y), Rotation2d.fromDegrees(180))
					)
				)
			)
		);

//			robot.getSwerve().getCommandsBuilder().driveToPose(
//				() -> new Pose2d(robot.getPoseEstimator().getEstimatedPose().getTranslation(), robot.getHeadingEstimator().getEstimatedHeading()),
//				() -> new Pose2d(
//					new Translation2d(15.794, 2.917), Rotation2d.fromRadians(2.675)
//				))

//		usedJoystick.X.whileTrue(robot.getSwerve().getCommandsBuilder().pointWheels(Rotation2d.fromDegrees(90), true));

		usedJoystick.POV_UP.whileTrue(robot.getSwerve().getCommandsBuilder().turnToHeading(Rotation2d.fromDegrees(180)));
		usedJoystick.POV_DOWN.whileTrue(
			robot.getSwerve()
				.getCommandsBuilder()
				.turnToHeading(Rotation2d.fromDegrees(-17))
				.until(
					() -> robot.getSwerve()
						.isAtHeading(Rotation2d.fromDegrees(-17), Tolerances.SWERVE_HEADING, Tolerances.ROTATION_VELOCITY_DEADBAND)
				)
		);

		usedJoystick.POV_LEFT
			.whileTrue(robot.getSwerve().getCommandsBuilder().turnToHeading(Rotation2d.fromDegrees(-17), RotateAxis.FRONT_LEFT_MODULE));
		usedJoystick.POV_RIGHT
			.whileTrue(robot.getSwerve().getCommandsBuilder().turnToHeading(Rotation2d.fromDegrees(180), RotateAxis.BACK_RIGHT_MODULE));

		usedJoystick.Y.whileTrue(
			robot.getSwerve()
				.getCommandsBuilder()
				.driveByState(
					() -> usedJoystick.getAxisValue(Axis.LEFT_Y),
					() -> usedJoystick.getAxisValue(Axis.LEFT_X),
					() -> usedJoystick.getSensitiveAxisValue(Axis.RIGHT_X),
					SwerveState.DEFAULT_DRIVE.withAimAssist(AimAssist.COOL_AMP)
				)
		);
		usedJoystick.L1.whileTrue(
			robot.getSwerve()
				.getCommandsBuilder()
				.driveByState(
					() -> usedJoystick.getAxisValue(Axis.LEFT_Y),
					() -> usedJoystick.getAxisValue(Axis.LEFT_X),
					() -> usedJoystick.getSensitiveAxisValue(Axis.RIGHT_X),
					SwerveState.DEFAULT_DRIVE.withDriveRelative(DriveRelative.ROBOT_RELATIVE).withAimAssist(AimAssist.NOTE)
				)
		);
		usedJoystick.R1.whileTrue(
			robot.getSwerve()
				.getCommandsBuilder()
				.driveByState(
					() -> usedJoystick.getAxisValue(Axis.LEFT_Y),
					() -> usedJoystick.getAxisValue(Axis.LEFT_X),
					() -> usedJoystick.getSensitiveAxisValue(Axis.RIGHT_X),
					SwerveState.DEFAULT_DRIVE.withAimAssist(AimAssist.SPEAKER)
				)
		);

		usedJoystick.getAxisAsButton(Axis.RIGHT_TRIGGER)
			.whileTrue(
				robot.getSwerve()
					.getCommandsBuilder()
					.driveByState(
						() -> usedJoystick.getAxisValue(Axis.LEFT_Y),
						() -> usedJoystick.getAxisValue(Axis.LEFT_X),
						() -> usedJoystick.getSensitiveAxisValue(Axis.RIGHT_X),
						() -> SwerveState.DEFAULT_DRIVE.withRotateAxis(robot.getSwerve().getStateHandler().getFarRightRotateAxis())
					)
			);
		usedJoystick.getAxisAsButton(Axis.LEFT_TRIGGER)
			.whileTrue(
				robot.getSwerve()
					.getCommandsBuilder()
					.driveByState(
						() -> usedJoystick.getAxisValue(Axis.LEFT_Y),
						() -> usedJoystick.getAxisValue(Axis.LEFT_X),
						() -> usedJoystick.getSensitiveAxisValue(Axis.RIGHT_X),
						() -> SwerveState.DEFAULT_DRIVE.withRotateAxis(robot.getSwerve().getStateHandler().getFarLeftRotateAxis())
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
				.driveToPose(robot.getPoseEstimator()::getEstimatedPose, () -> new Pose2d(4, 4, Rotation2d.fromDegrees(17)))
				.until(() -> robot.getSuperStructure().isAtPose(new Pose2d(4, 4, Rotation2d.fromDegrees(17))))
		);
		usedJoystick.START.whileTrue(
			robot.getSwerve()
				.getCommandsBuilder()
				.driveToPose(robot.getPoseEstimator()::getEstimatedPose, () -> new Pose2d(6, 6, Rotation2d.fromDegrees(90)))
				.until(() -> robot.getSuperStructure().isAtPose(new Pose2d(6, 6, Rotation2d.fromDegrees(90))))
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
				.driveToPose(robot.getPoseEstimator()::getEstimatedPose, () -> new Pose2d(1, 8, Rotation2d.fromDegrees(90)))
				.until(() -> robot.getSuperStructure().isAtPose(new Pose2d(1, 8, Rotation2d.fromDegrees(90))))
		);
		usedJoystick.X.whileTrue(
			robot.getSwerve()
				.getCommandsBuilder()
				.driveToPose(robot.getPoseEstimator()::getEstimatedPose, () -> new Pose2d(6, 6, Rotation2d.fromDegrees(117)))
				.until(() -> robot.getSuperStructure().isAtPose(new Pose2d(6, 6, Rotation2d.fromDegrees(117))))
		);
		usedJoystick.Y.whileTrue(
			robot.getSwerve()
				.getCommandsBuilder()
				.driveToPose(robot.getPoseEstimator()::getEstimatedPose, () -> new Pose2d(7, 2, Rotation2d.fromDegrees(180)))
				.until(() -> robot.getSuperStructure().isAtPose(new Pose2d(7, 2, Rotation2d.fromDegrees(180))))
		);
		usedJoystick.B.whileTrue(
			robot.getSwerve()
				.getCommandsBuilder()
				.driveToPose(robot.getPoseEstimator()::getEstimatedPose, () -> new Pose2d(16, 6, Rotation2d.fromDegrees(-75)))
				.until(() -> robot.getSuperStructure().isAtPose(new Pose2d(16, 6, Rotation2d.fromDegrees(-75))))
		);
		usedJoystick.START.whileTrue(
			robot.getSwerve()
				.getCommandsBuilder()
				.driveToPose(robot.getPoseEstimator()::getEstimatedPose, () -> new Pose2d(12, 8, Rotation2d.fromDegrees(14)))
				.until(() -> robot.getSuperStructure().isAtPose(new Pose2d(12, 8, Rotation2d.fromDegrees(14))))
		);
		usedJoystick.BACK.whileTrue(
			robot.getSwerve()
				.getCommandsBuilder()
				.driveToPose(robot.getPoseEstimator()::getEstimatedPose, () -> new Pose2d(10, 4, Rotation2d.fromDegrees(140)))
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
