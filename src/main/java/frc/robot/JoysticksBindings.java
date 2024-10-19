package frc.robot;

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

		usedJoystick.R1.onTrue(robot.getStatesMotionPlanner().setState(RobotState.INTAKE));
		usedJoystick.getAxisAsButton(Axis.RIGHT_TRIGGER).onTrue(robot.getStatesMotionPlanner().setState(RobotState.SPEAKER));

		usedJoystick.POV_RIGHT.onTrue(robot.getStatesMotionPlanner().setState(RobotState.TRANSFER_ARM_TO_SHOOTER));
		usedJoystick.POV_LEFT.onTrue(robot.getStatesMotionPlanner().setState(RobotState.TRANSFER_SHOOTER_TO_ARM));

		usedJoystick.L1.onTrue(robot.getStatesMotionPlanner().setState(RobotState.ARM_INTAKE));
		usedJoystick.getAxisAsButton(Axis.LEFT_TRIGGER).onTrue(robot.getStatesMotionPlanner().setState(RobotState.AMP));

		usedJoystick.Y.onTrue(robot.getStatesMotionPlanner().setState(RobotState.PASSING));
		usedJoystick.A.onTrue(robot.getStatesMotionPlanner().setState(RobotState.IDLE));
		usedJoystick.X.onTrue(robot.getStatesMotionPlanner().setState(RobotState.INTAKE_OUTTAKE));
		usedJoystick.B.onTrue(new InstantCommand(() -> {
			robot.getPoseEstimator().resetHeadingOffset(new Rotation2d());
		}));
	}

	private static void secondJoystickButtons(Robot robot) {
		SmartJoystick usedJoystick = SECOND_JOYSTICK;
		// bindings...
	}

	private static void thirdJoystickButtons(Robot robot) {
		SmartJoystick usedJoystick = THIRD_JOYSTICK;
		// bindings...
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
