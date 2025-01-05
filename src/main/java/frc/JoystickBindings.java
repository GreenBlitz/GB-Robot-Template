package frc;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Robot;
import frc.joysticks.SmartJoystick;
import frc.utils.utilcommands.InitExecuteCommand;

import java.util.function.Function;

public class JoystickBindings {

	public enum BindSet {

		NONE,
		EMPTY,
		SWERVE,
		SECOND,
		TESTING

	}

	public static void configureBindings(SmartJoystick joystick, Robot robot) {
		applySecondBindings(joystick);
		applySwerveBindings(joystick);
	}

	private static Trigger bind(
		SmartJoystick joystick,
		Trigger bind,
		BindSet bindSetRequirement
	) {
		return bind.and(() -> joystick.getBindSet() == bindSetRequirement);
	}

	private static Trigger bind(
		SmartJoystick joystick,
		BindSet bindSetRequirement
	) {
		return new Trigger(() -> bindSetRequirement == joystick.getBindSet());
	}

	private static void applySwerveBindings(SmartJoystick joystick){
		bind(joystick, joystick.A, BindSet.SWERVE).whileTrue(new RunCommand(() -> System.out.println("sweeeeeerve")));
	}

	private static void applySecondBindings(SmartJoystick joystick){
		bind(joystick, joystick.A, BindSet.SECOND).whileTrue(new RunCommand(() -> System.out.println("secoooond")));

	}

}
