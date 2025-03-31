package frc;

import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.joysticks.BindSet;
import frc.robot.Robot;
import frc.joysticks.SmartJoystick;

public class JoystickBindings {

	public static void configureBindings(SmartJoystick joystick, Robot robot) {
		applySwerveBindings(joystick);
		applySecondBindings(joystick);
	}

	private static Trigger bindSetTrigger(SmartJoystick joystick, Trigger bind, BindSet bindSetRequirement) {
		return bind.and(bindSetTrigger(joystick, bindSetRequirement));
	}

	private static Trigger bindSetTrigger(SmartJoystick joystick, BindSet bindSetRequirement) {
		return new Trigger(() -> bindSetRequirement == joystick.getBindSet());
	}

	private static void applySwerveBindings(SmartJoystick joystick) {
		bindSetTrigger(joystick, BindSet.SWERVE).whileTrue(new RunCommand(() -> System.out.println(joystick.getLogPath())));
		bindSetTrigger(joystick, joystick.A, BindSet.SWERVE).whileTrue(new RunCommand(() -> System.out.println("sweeeerv")));
	}

	private static void applySecondBindings(SmartJoystick joystick) {
		bindSetTrigger(joystick, joystick.A, BindSet.SECOND).whileTrue(new RunCommand(() -> System.out.println("Secoooond")));
	}


}
