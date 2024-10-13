package frc.robot.subsystems.wrist.factory;

import frc.robot.Robot;
import frc.robot.subsystems.wrist.WristStuff;

public class WristFactory {

	public static WristStuff create(String logPath) {
		return switch (Robot.ROBOT_TYPE) {
			case REAL -> RealWristConstants.generateWristStuff(logPath);
			case SIMULATION -> null;
		};
	}

}
