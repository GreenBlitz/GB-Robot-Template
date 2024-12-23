package frc.robot.joystickManager;

import frc.utils.joysticks.JoystickPorts;
import frc.utils.joysticks.SmartJoystick;

public class JoystickManager {

	private static SmartJoystick mainJoystick = null;
	private static SmartJoystick secondJoystick = null;
	private static SmartJoystick thirdJoystick = null;
	private static SmartJoystick fourthJoystick = null;
	private static SmartJoystick fifthJoystick = null;
	private static SmartJoystick sixthJoystick = null;

	public static void setMainJoystickState(JoystickState state) {
		if (state == JoystickState.NONE) {
			mainJoystick = null;
		} else if (mainJoystick == null) {
			mainJoystick = new SmartJoystick(JoystickPorts.MAIN, state);
		} else {
			mainJoystick.setState(state);
		}
	}

	public static void setSecondJoystickState(JoystickState state) {
		if (state == JoystickState.NONE) {
			secondJoystick = null;
		} else if (secondJoystick == null) {
			secondJoystick = new SmartJoystick(JoystickPorts.SECOND, state);
		} else {
			secondJoystick.setState(state);
		}
	}

	public static void setThirdJoystickState(JoystickState state) {
		if (state == JoystickState.NONE) {
			thirdJoystick = null;
		} else if (thirdJoystick == null) {
			thirdJoystick = new SmartJoystick(JoystickPorts.THIRD, state);
		} else {
			thirdJoystick.setState(state);
		}
	}

	public static void setFourthJoystickState(JoystickState state) {
		if (state == JoystickState.NONE) {
			fourthJoystick = null;
		} else if (fourthJoystick == null) {
			fourthJoystick = new SmartJoystick(JoystickPorts.FOURTH, state);
		} else {
			fourthJoystick.setState(state);
		}
	}

	public static void setFifthJoystickState(JoystickState state) {
		if (state == JoystickState.NONE) {
			fifthJoystick = null;
		} else if (fifthJoystick == null) {
			fifthJoystick = new SmartJoystick(JoystickPorts.FIFTH, state);
		} else {
			fifthJoystick.setState(state);
		}
	}

	public static void setSixthJoystickState(JoystickState state) {
		if (state == JoystickState.NONE) {
			sixthJoystick = null;
		} else if (sixthJoystick == null) {
			sixthJoystick = new SmartJoystick(JoystickPorts.SIXTH, state);
		} else {
			sixthJoystick.setState(state);
		}
	}

	public static SmartJoystick getFifthJoystick() {
		if (fifthJoystick == null) {
			setFifthJoystickState(JoystickState.EMPTY);
		}
		return fifthJoystick;
	}

}
