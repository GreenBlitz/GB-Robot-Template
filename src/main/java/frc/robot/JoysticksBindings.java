package frc.robot;

import frc.robot.subsystems.lifter.LifterStateHandler;
import frc.robot.subsystems.solenoid.SolenoidStateHandler;
import frc.robot.superstructure.climb.ClimbState;
import frc.robot.superstructure.climb.ClimbStateHandler;
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

		ClimbStateHandler climbStateHandler = new ClimbStateHandler(
			new LifterStateHandler(robot.getLifter()),
			new SolenoidStateHandler(robot.getSolenoid())
		);

		usedJoystick.B.onTrue(climbStateHandler.setState(ClimbState.STOP));
		usedJoystick.X.onTrue(climbStateHandler.setState(ClimbState.EXTEND));
		usedJoystick.A.onTrue(climbStateHandler.setState(ClimbState.RETRACT));
		// bindings...
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
