package frc;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.joysticks.Axis;
import frc.joysticks.JoystickPorts;
import frc.joysticks.XboxSmartJoystick;
import frc.robot.Robot;
import frc.robot.subsystems.swerve.ChassisPowers;
import frc.utils.brakestate.BrakeMode;
import frc.utils.brakestate.BrakeStateManager;

public class JoysticksBindings {

	private static final XboxSmartJoystick MAIN_JOYSTICK = new XboxSmartJoystick(JoystickPorts.MAIN, true);
	private static final XboxSmartJoystick SECOND_JOYSTICK = new XboxSmartJoystick(JoystickPorts.SECOND);
	private static final XboxSmartJoystick THIRD_JOYSTICK = new XboxSmartJoystick(JoystickPorts.THIRD);
	private static final XboxSmartJoystick FOURTH_JOYSTICK = new XboxSmartJoystick(JoystickPorts.FOURTH);
	private static final XboxSmartJoystick FIFTH_JOYSTICK = new XboxSmartJoystick(JoystickPorts.FIFTH);
	private static final XboxSmartJoystick SIXTH_JOYSTICK = new XboxSmartJoystick(JoystickPorts.SIXTH);

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
		XboxSmartJoystick usedJoystick = MAIN_JOYSTICK;
		// bindings...
		usedJoystick.X.onTrue(new InstantCommand(() -> BrakeStateManager.setBrakeMode(BrakeMode.COAST)));
		usedJoystick.A.onTrue(new InstantCommand(() -> BrakeStateManager.setBrakeMode(BrakeMode.BRAKE)));
	}

	private static void secondJoystickButtons(Robot robot) {
		XboxSmartJoystick usedJoystick = SECOND_JOYSTICK;
		// bindings...
	}

	private static void thirdJoystickButtons(Robot robot) {
		XboxSmartJoystick usedJoystick = THIRD_JOYSTICK;
		// bindings...
	}

	private static void fourthJoystickButtons(Robot robot) {
		XboxSmartJoystick usedJoystick = FOURTH_JOYSTICK;
		// bindings...
	}

	private static void fifthJoystickButtons(Robot robot) {
		XboxSmartJoystick usedJoystick = FIFTH_JOYSTICK;
		// bindings...
	}

	private static void sixthJoystickButtons(Robot robot) {
		XboxSmartJoystick usedJoystick = SIXTH_JOYSTICK;
		// bindings...
	}

}
