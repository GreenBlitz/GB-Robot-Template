package frc.robot;

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
		robot.getSwerve().setDefaultCommand(robot.getSwerve().getCommandsBuilder().driveBySavedState(() -> 0, () -> 0, () -> 0));

		usedJoystick.A.onTrue(robot.getSuperstructure().setState(RobotState.IDLE));
		usedJoystick.B.onTrue(robot.getSuperstructure().setState(RobotState.SHOOTER_INTAKE));
		usedJoystick.X.onTrue(robot.getSuperstructure().setState(RobotState.ARM_INTAKE));
		usedJoystick.Y.onTrue(robot.getSuperstructure().setState(RobotState.PRE_AMP));
		usedJoystick.POV_UP.onTrue(robot.getSuperstructure().setState(RobotState.PRE_SPEAKER));
		usedJoystick.POV_LEFT.onTrue(robot.getSuperstructure().setState(RobotState.TRANSFER_SHOOTER_TO_ARM));
		usedJoystick.POV_RIGHT.onTrue(robot.getSuperstructure().setState(RobotState.TRANSFER_ARM_TO_SHOOTER));

		usedJoystick.L1.whileTrue(robot.getRoller().getCommandsBuilder().setPower(() -> usedJoystick.getAxisValue(Axis.LEFT_Y)));
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
