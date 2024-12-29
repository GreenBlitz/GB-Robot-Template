package frc.robot.joystickManager;

import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.Robot;
import frc.joysticks.SmartJoystick;

public class JoystickBindings {

	public static void configureBindings(SmartJoystick joystick, Robot robot) {
		applySwerveBindnigs(joystick, robot);
		applySecondBindings(joystick, robot);
	}

	private static void applySwerveBindnigs(SmartJoystick joystick, Robot robot) {
		joystick.A.and(() -> joystick.getState() == JoystickState.SWERVE).whileTrue(new RunCommand(() -> System.out.println("swerve")));
	}

	private static void applySecondBindings(SmartJoystick joystick, Robot robot) {
		joystick.A.and(() -> joystick.getState() == JoystickState.SECOND).whileTrue(new RunCommand(() -> System.out.println("second")));
	}

}
