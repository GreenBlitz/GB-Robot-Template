package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.superstructure.RobotState;
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

	public static SmartJoystick getMainJoystick() {
		return MAIN_JOYSTICK;
	}

	public static SmartJoystick getSecondJoystick() {
		return SECOND_JOYSTICK;
	}

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

		robot.getSwerve()
			.setDefaultCommand(
				robot.getSwerve()
					.getCommandsBuilder()
					.driveBySavedState(
						() -> usedJoystick.getAxisValue(Axis.LEFT_Y),
						() -> usedJoystick.getAxisValue(Axis.LEFT_X),
						() -> usedJoystick.getAxisValue(Axis.RIGHT_X)
					)
			);

		usedJoystick.R1.onTrue(robot.getSuperstructure().setState(RobotState.INTAKE));
		usedJoystick.L1.onTrue(robot.getSuperstructure().setState(RobotState.SPEAKER));
		usedJoystick.getAxisAsButton(Axis.LEFT_TRIGGER).onTrue(robot.getSuperstructure().setState(RobotState.AMP));
//		usedJoystick.Y.onTrue(new InstantCommand(
//				() -> robot.getPoseEstimator().resetPose(robot.getPoseEstimator().getVisionPose().get())));
//		usedJoystick.getAxisAsButton(Axis.RIGHT_TRIGGER).onTrue(robot.getSuperstructure().setState(RobotState.INTAKE_OUTTAKE));
	}

	private static void secondJoystickButtons(Robot robot) {
		SmartJoystick usedJoystick = SECOND_JOYSTICK;
		// bindings...

		usedJoystick.A.onTrue(robot.getSuperstructure().setState(RobotState.IDLE));
		usedJoystick.R1.onTrue(robot.getSuperstructure().setState(RobotState.PRE_SPEAKER));
		usedJoystick.L1.onTrue(robot.getSuperstructure().setState(RobotState.PRE_AMP));
		usedJoystick.BACK.onTrue(robot.getSuperstructure().setState(RobotState.TRANSFER_SHOOTER_ELEVATOR));
		usedJoystick.START.onTrue(robot.getSuperstructure().setState(RobotState.TRANSFER_ELEVATOR_SHOOTER));
		usedJoystick.POV_DOWN.onTrue(robot.getSuperstructure().setState(RobotState.INTAKE_OUTTAKE));
		usedJoystick.POV_UP.onTrue(robot.getSuperstructure().setState(RobotState.SHOOTER_OUTTAKE));
	}

	private static void thirdJoystickButtons(Robot robot) {
		SmartJoystick usedJoystick = THIRD_JOYSTICK;
		// bindings...

//		usedJoystick.BACK.onTrue(robot.getPivot().getCommandsBuilder().goToPosition(Rotation2d.fromDegrees(90)));
//
//		usedJoystick.POV_DOWN.whileTrue(
//				robot.getSwerve().getCommandsBuilder().turnToHeading(Rotation2d.fromDegrees(0)));
//		usedJoystick.POV_UP.whileTrue(
//				robot.getSwerve().getCommandsBuilder().turnToHeading(Rotation2d.fromDegrees(180)));
//
//		usedJoystick.Y.whileTrue(robot.getSwerve().getCommandsBuilder().driveToPose(
//				() -> robot.getPoseEstimator().getEstimatedPose(),
//                Pose2d::new,
//				(pose2d) -> robot.getPoseEstimator().isAtPose(pose2d, robot.getSwerve())
//		));
//
//		usedJoystick.A.whileTrue(robot.getSwerve().getCommandsBuilder().driveToPose(
//				() -> robot.getPoseEstimator().getEstimatedPose(),
//				() -> new Pose2d(2, 1, Rotation2d.fromDegrees(90)),
//				(pose2d) -> robot.getPoseEstimator().isAtPose(pose2d, robot.getSwerve())
//		));
//
//		usedJoystick.B.onTrue(
//				new InstantCommand(() -> robot.getPoseEstimator().resetPose(new Pose2d(1,1,new Rotation2d()))));
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
