package frc;

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

		private final int index;

		BindSet(int index) {
			this.index = index;
		}

		public int getIndex() {
			return index;
		}

		public static BindSet getBindSetByIndex(int index) {
			return switch (index) {
				case 0 -> NONE;
				case 1 -> EMPTY;
				case 2 -> SWERVE;
				case 3 -> SECOND;
				case 4 -> TESTING;
				default -> EMPTY;
			};
		}

	}

	public static void configureBindings(SmartJoystick joystick, Robot robot) {}

	private static Trigger bindSetTrigger(SmartJoystick joystick, Trigger bind, BindSet bindSetRequirement) {
		return bind.and(() -> bindSetRequirement == joystick.getBindSet());
	}

	private static Trigger bindSetTrigger(SmartJoystick joystick, BindSet bindSetRequirement) {
		return new Trigger(() -> bindSetRequirement == joystick.getBindSet());
	}


}
