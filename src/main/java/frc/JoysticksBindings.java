package frc;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.joysticks.Axis;
import frc.joysticks.JoystickPorts;
import frc.joysticks.SmartJoystick;
import frc.robot.Robot;
import frc.robot.subsystems.swerve.factories.constants.RealSwerveConstants;
import frc.robot.subsystems.swerve.states.LoopMode;
import frc.robot.subsystems.swerve.states.SwerveState;

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

		usedJoystick.POV_UP.onTrue(robot.getSwerve().getCommandsBuilder().turnToHeading(Rotation2d.fromDegrees(0)));
		usedJoystick.POV_RIGHT.onTrue(robot.getSwerve().getCommandsBuilder().turnToHeading(Rotation2d.fromDegrees(90)));
		usedJoystick.POV_DOWN.onTrue(robot.getSwerve().getCommandsBuilder().turnToHeading(Rotation2d.fromDegrees(180)));
		usedJoystick.POV_LEFT.onTrue(robot.getSwerve().getCommandsBuilder().turnToHeading(Rotation2d.fromDegrees(270)));

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
	}

	private static void secondJoystickButtons(Robot robot) {
		SmartJoystick usedJoystick = SECOND_JOYSTICK;
		// bindings...
		usedJoystick.A.whileTrue(robot.getSwerve().getCommandsBuilder().wheelRadiusCalibration());
		usedJoystick.B.whileTrue(robot.getSwerve().getCommandsBuilder().steerCalibration(true, SysIdRoutine.Direction.kForward));
		usedJoystick.Y.whileTrue(robot.getSwerve().getCommandsBuilder().driveCalibration(true, SysIdRoutine.Direction.kForward));

		usedJoystick.POV_DOWN.whileTrue(robot.getSwerve().getCommandsBuilder().drive(() -> 0.2, () -> 0, () -> 0));
		usedJoystick.POV_LEFT.whileTrue(robot.getSwerve().getCommandsBuilder().drive(() -> 0.5, () -> 0, () -> 0));
		usedJoystick.POV_RIGHT.whileTrue(robot.getSwerve().getCommandsBuilder().drive(() -> -0.2, () -> 0, () -> 0));
		usedJoystick.POV_UP.whileTrue(robot.getSwerve().getCommandsBuilder().drive(() -> -0.5, () -> 0, () -> 0));
	}

	private static void thirdJoystickButtons(Robot robot) {
		SmartJoystick usedJoystick = THIRD_JOYSTICK;
		// bindings...
		usedJoystick.START.whileTrue(robot.getSwerve().getCommandsBuilder().drive(() -> 0, () -> 0, () -> 0));
		usedJoystick.Y.whileTrue(
			robot.getSwerve()
				.getCommandsBuilder()
				.driveByState(
					() -> 0.5 / RealSwerveConstants.VELOCITY_AT_12_VOLTS_METERS_PER_SECOND,
					() -> 0,
					() -> 0,
					SwerveState.DEFAULT_DRIVE.withLoopMode(LoopMode.CLOSED)
				)
		);


		usedJoystick.POV_DOWN.whileTrue(robot.getSwerve().getCommandsBuilder().wheelRadiusCalibration());
	}

	private static void fourthJoystickButtons(Robot robot) {
		SmartJoystick usedJoystick = FOURTH_JOYSTICK;
		// bindings...
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
