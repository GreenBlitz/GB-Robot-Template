package frc.robot.subsystems.elbow.factory;

import frc.robot.Robot;
import frc.robot.subsystems.elbow.ElbowStuff;

public class ElbowFactory {

	public static ElbowStuff create(String logPath) {
		return switch (Robot.ROBOT_TYPE) {
			case REAL -> RealElbowConstants.generateElbowStuff(logPath);
			case SIMULATION -> null;
		};
	}

}
