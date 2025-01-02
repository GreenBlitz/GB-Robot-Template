package frc;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Robot;
import frc.joysticks.SmartJoystick;


public class JoystickBindings {

	public static void configureBindings(SmartJoystick joystick, Robot robot) {}

	private static void bind(SmartJoystick joystick, Trigger bind, JoystickBindSet bindSetRequirement, Command command) {
		bind.and(() -> joystick.getBindSet() == bindSetRequirement).onTrue(command);
	}

	// bind functions:

	public enum JoystickBindSet {

		NONE,
		EMPTY,
		SWERVE,
		SECOND,
		TEST;
	}

}
