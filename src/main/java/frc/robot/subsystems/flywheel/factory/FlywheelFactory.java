package frc.robot.subsystems.flywheel.factory;

import frc.robot.Robot;
import frc.robot.subsystems.flywheel.FlywheelStuff;

public class FlywheelFactory {

	public static FlywheelStuff create(String logPath) {
		return switch (Robot.ROBOT_TYPE) {
			case REAL -> RealFlywheelConstants.generateFlywheelStuff(logPath);
			case SIMULATION -> null;
		};
	}

}
