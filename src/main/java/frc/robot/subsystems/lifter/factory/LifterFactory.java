package frc.robot.subsystems.lifter.factory;

import frc.robot.Robot;
import frc.robot.subsystems.lifter.LifterComponents;

public class LifterFactory {

	public static LifterComponents create(String logPath) {
		return switch (Robot.ROBOT_TYPE) {
			case REAL -> LifterRealConstants.generateLifterComponents(logPath);
			case SIMULATION -> null;
		};
	}

}
