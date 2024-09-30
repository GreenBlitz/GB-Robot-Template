package frc.robot.subsystems.roller.factory;

import frc.robot.Robot;
import frc.robot.subsystems.roller.RollerComponents;

public class RollerFactory {

	public static RollerComponents create(String logPath) {
		return switch (Robot.ROBOT_TYPE) {
			case REAL -> RealRollerConstants.generateIntakeStuff(logPath);
			case SIMULATION -> null;
		};
	}

}
