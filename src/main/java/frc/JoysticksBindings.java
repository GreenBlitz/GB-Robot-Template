package frc;

import frc.joysticks.Axis;
import frc.joysticks.JoystickPorts;
import frc.joysticks.SmartJoystickPlayStation;
import frc.robot.Robot;
import frc.robot.subsystems.swerve.ChassisPowers;

public class JoysticksBindings {

	private static final SmartJoystickPlayStation MAIN_JOYSTICK = new SmartJoystickPlayStation(JoystickPorts.MAIN, true);
	private static final SmartJoystickPlayStation SECOND_JOYSTICK = new SmartJoystickPlayStation(JoystickPorts.SECOND);
	private static final SmartJoystickPlayStation THIRD_JOYSTICK = new SmartJoystickPlayStation(JoystickPorts.THIRD);
	private static final SmartJoystickPlayStation FOURTH_JOYSTICK = new SmartJoystickPlayStation(JoystickPorts.FOURTH);
	private static final SmartJoystickPlayStation FIFTH_JOYSTICK = new SmartJoystickPlayStation(JoystickPorts.FIFTH);
	private static final SmartJoystickPlayStation SIXTH_JOYSTICK = new SmartJoystickPlayStation(JoystickPorts.SIXTH);

	private static final ChassisPowers chassisDriverInputs = new ChassisPowers();

	public static void configureBindings(Robot robot) {
		robot.getSwerve().setDriversPowerInputs(chassisDriverInputs);

		mainJoystickButtons(robot);
		secondJoystickButtons(robot);
		thirdJoystickButtons(robot);
		fourthJoystickButtons(robot);
		fifthJoystickButtons(robot);
		sixthJoystickButtons(robot);
	}

	public static void updateChassisDriverInputs() {
		if (MAIN_JOYSTICK.isConnected()) {
			chassisDriverInputs.xPower = MAIN_JOYSTICK.getAxisValue(Axis.LEFT_Y);
			chassisDriverInputs.yPower = MAIN_JOYSTICK.getAxisValue(Axis.LEFT_X);
			chassisDriverInputs.rotationalPower = MAIN_JOYSTICK.getAxisValue(Axis.RIGHT_X);
		} else if (THIRD_JOYSTICK.isConnected()) {
			chassisDriverInputs.xPower = THIRD_JOYSTICK.getAxisValue(Axis.LEFT_Y);
			chassisDriverInputs.yPower = THIRD_JOYSTICK.getAxisValue(Axis.LEFT_X);
			chassisDriverInputs.rotationalPower = THIRD_JOYSTICK.getAxisValue(Axis.RIGHT_X);
		} else {
			chassisDriverInputs.xPower = 0;
			chassisDriverInputs.yPower = 0;
			chassisDriverInputs.rotationalPower = 0;
		}
	}

	private static void mainJoystickButtons(Robot robot) {
		SmartJoystickPlayStation usedJoystick = MAIN_JOYSTICK;
		// bindings...
	}

	private static void secondJoystickButtons(Robot robot) {
		SmartJoystickPlayStation usedJoystick = SECOND_JOYSTICK;
		// bindings...
	}

	private static void thirdJoystickButtons(Robot robot) {
		SmartJoystickPlayStation usedJoystick = THIRD_JOYSTICK;
		// bindings...
	}

	private static void fourthJoystickButtons(Robot robot) {
		SmartJoystickPlayStation usedJoystick = FOURTH_JOYSTICK;
		// bindings...
	}

	private static void fifthJoystickButtons(Robot robot) {
		SmartJoystickPlayStation usedJoystick = FIFTH_JOYSTICK;
		// bindings...
	}

	private static void sixthJoystickButtons(Robot robot) {
		SmartJoystickPlayStation usedJoystick = SIXTH_JOYSTICK;
		// bindings...
	}

}
