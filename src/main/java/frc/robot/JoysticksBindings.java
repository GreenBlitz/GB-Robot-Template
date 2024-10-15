package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.superstructure.RobotState;
import frc.robot.subsystems.flywheel.FlywheelState;
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

	public static SmartJoystick getThirdJoystick() {
		return THIRD_JOYSTICK;
	}

	public static SmartJoystick getFourthJoystick() {
		return FOURTH_JOYSTICK;
	}

	public static SmartJoystick getFifthJoystick() {
		return FIFTH_JOYSTICK;
	}

	public static SmartJoystick getSixthJoystick() {
		return SIXTH_JOYSTICK;
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

		Pose2d old = robot.getPoseEstimator().getEstimatedPose();
		usedJoystick.Y
			.onTrue(new InstantCommand(() -> robot.getPoseEstimator().resetPose(new Pose2d(old.getX(), old.getY(), Rotation2d.fromDegrees(0)))));
		usedJoystick.B.onTrue(new InstantCommand(() -> robot.getPoseEstimator().resetPoseByLimelight()));

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
		// for aim assist test
		usedJoystick.POV_UP.onTrue(robot.getStatesMotionPlanner().setState(RobotState.ARM_INTAKE));
		usedJoystick.POV_DOWN.onTrue(robot.getStatesMotionPlanner().setState(RobotState.PASSING));
		usedJoystick.getAxisAsButton(Axis.RIGHT_TRIGGER).onTrue(robot.getStatesMotionPlanner().setState(RobotState.SPEAKER));
		usedJoystick.R1.onTrue(robot.getStatesMotionPlanner().setState(RobotState.PRE_SPEAKER));
		usedJoystick.L1.onTrue(robot.getStatesMotionPlanner().setState(RobotState.INTAKE));
		usedJoystick.A.onTrue(robot.getStatesMotionPlanner().setState(RobotState.IDLE));
		usedJoystick.getAxisAsButton(Axis.LEFT_TRIGGER).onTrue(robot.getStatesMotionPlanner().setState(RobotState.INTAKE_OUTTAKE));
	}

	private static void secondJoystickButtons(Robot robot) {
		SmartJoystick usedJoystick = SECOND_JOYSTICK;
		// bindings...

		usedJoystick.getAxisAsButton(Axis.LEFT_TRIGGER, 0.1)
			.whileTrue(robot.getFunnel().getCommandsBuilder().setPower(() -> -usedJoystick.getAxisValue(Axis.LEFT_TRIGGER) * 0.5));

		usedJoystick.getAxisAsButton(Axis.RIGHT_TRIGGER, 0.1)
			.whileTrue(
				robot.getFunnel()
					.getCommandsBuilder()
					.setPower(() -> usedJoystick.getAxisValue(Axis.RIGHT_TRIGGER) * 0.7)
					.alongWith(robot.getIntake().getCommandsBuilder().setPower(() -> usedJoystick.getAxisValue(Axis.RIGHT_TRIGGER) * 0.4))
			);

		usedJoystick.R1.onTrue(
			robot.getFlywheel()
				.getCommandsBuilder()
				.setVelocities(FlywheelState.PRE_SPEAKER.getRightVelocity(), FlywheelState.PRE_SPEAKER.getLeftVelocity())
		);
		usedJoystick.L1.onTrue(
			robot.getFlywheel()
				.getCommandsBuilder()
				.setVelocities(FlywheelState.DEFAULT.getRightVelocity(), FlywheelState.DEFAULT.getLeftVelocity())
		);
	}

	private static void thirdJoystickButtons(Robot robot) {
		SmartJoystick usedJoystick = THIRD_JOYSTICK;


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
