package frc;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Robot;
import frc.joysticks.SmartJoystick;
import frc.utils.utilcommands.InitExecuteCommand;

public class JoystickBindings {

	public enum JoystickBindSet {

		NONE,
		EMPTY,
		SWERVE,
		SECOND,
		TESTING

	}

	public static void configureBindings(SmartJoystick joystick, Robot robot) {}

	private static void bind(SmartJoystick joystick, Trigger bind, JoystickBindSet bindSetRequirement, Command command) {
		bind.and(() -> joystick.getBindSet() == bindSetRequirement).onTrue(command);
	}

	private static void bind(SmartJoystick joystick, JoystickBindSet bindSetRequirement, Runnable initRun, Runnable executeRun) {
		new InitExecuteCommand(initRun, executeRun).onlyWhile(() -> bindSetRequirement == joystick.getBindSet());
	}

}
