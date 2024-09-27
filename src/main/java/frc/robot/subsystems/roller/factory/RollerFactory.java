package frc.robot.subsystems.roller.factory;

import frc.robot.Robot;
import frc.robot.subsystems.roller.RollerStuff;

public class RollerFactory {

	public static RollerStuff create(String logPath) {
		return switch (Robot.ROBOT_TYPE) {
			case REAL -> RealRollerConstants.generateIntakeStuff(logPath);
			case SIMULATION -> null;
		};
	}

}
