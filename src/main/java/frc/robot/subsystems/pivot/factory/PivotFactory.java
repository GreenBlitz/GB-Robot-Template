package frc.robot.subsystems.pivot.factory;

import frc.robot.Robot;
import frc.robot.subsystems.pivot.PivotStuff;

public class PivotFactory {

	public static PivotStuff create(String logPath) {
		return switch (Robot.ROBOT_TYPE) {
			case REAL -> RealPivotConstants.generatePivotStuff(logPath);
			case SIMULATION -> null;
		};
	}

}
