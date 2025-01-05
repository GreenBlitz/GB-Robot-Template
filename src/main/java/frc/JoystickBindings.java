package frc;

import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Robot;
import frc.joysticks.SmartJoystick;


public class JoystickBindings {

	public enum BindSet {

		NONE,
		EMPTY,
		SWERVE,
		SECOND,
		TESTING

	}

	public static void configureBindings(SmartJoystick joystick, Robot robot) {}

	private static Trigger bindSetTrigger(SmartJoystick joystick, Trigger bind, BindSet requiredBindSet) {
		return bind.and(() -> joystick.getBindSet() == requiredBindSet);
	}

	private static Trigger bindSetTrigger(SmartJoystick joystick, BindSet requiredBindSet) {
		return new Trigger(() -> requiredBindSet == joystick.getBindSet());
	}

}
