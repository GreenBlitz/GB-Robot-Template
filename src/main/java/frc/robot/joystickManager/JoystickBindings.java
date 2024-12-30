package frc.robot.joystickManager;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Robot;
import frc.joysticks.SmartJoystick;


public class JoystickBindings {

	public static void configureBindings(SmartJoystick joystick, Robot robot) {}

	private static void bind(SmartJoystick joystick, Trigger trigger, JoystickBindSet bindSetRequirement, Command command) {
		trigger.onTrue(command).and(() -> joystick.getBindSet() == bindSetRequirement);
	}

}
