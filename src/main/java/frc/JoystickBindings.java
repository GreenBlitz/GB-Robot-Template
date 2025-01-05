package frc;

import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Robot;
import frc.joysticks.SmartJoystick;

public class JoystickBindings {

	public enum BindSet {

		NONE(0),
		EMPTY(1),
		SWERVE(2),
		SECOND(3),
		TESTING(4);

		private final int bindSet;

		BindSet(int bindSet) {
			this.bindSet = bindSet;
		}

		public int getBindSet() {
			return bindSet;
		}

		public static BindSet intToBindSet(int bindSet) {
			return switch (bindSet) {
				case 0 -> NONE;
				case 1 -> EMPTY;
				case 2 -> SWERVE;
				case 3 -> SECOND;
				case 4 -> TESTING;
				default -> EMPTY;
			};
		}

	}

	public static void configureBindings(SmartJoystick joystick, Robot robot) {
		applySecondBindings(joystick);
		applySwerveBindings(joystick);
	}

	private static Trigger bindSetTrigger(SmartJoystick joystick, Trigger bind, BindSet bindSetRequirement) {
		return bind.and(() -> joystick.getBindSet() == bindSetRequirement);
	}

	private static Trigger bindSetTrigger(SmartJoystick joystick, BindSet bindSetRequirement) {
		return new Trigger(() -> bindSetRequirement == joystick.getBindSet());
	}

	private static void applySwerveBindings(SmartJoystick joystick) {
		bindSetTrigger(joystick, BindSet.SWERVE).whileTrue(new RunCommand(() -> System.out.println(joystick.getLogPath())));
	}

	private static void applySecondBindings(SmartJoystick joystick) {
		bindSetTrigger(joystick, BindSet.SWERVE).whileTrue(new RunCommand(() -> System.out.println(joystick.getLogPath())));
	}


}
