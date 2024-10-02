package frc.robot.subsystems.flywheel.factory;

import frc.robot.Robot;
import frc.robot.subsystems.flywheel.BottomFlywheelComponents;
import frc.robot.subsystems.flywheel.TopFlywheelComponents;

public class FlywheelFactory {

	public static TopFlywheelComponents createTopFlywheelComponents(String logPath) {
		return switch (Robot.ROBOT_TYPE) {
			case REAL -> RealFlywheelConstants.generateTopFlywheelComponents(logPath, true);
			case SIMULATION -> null;
		};
	}

	public static BottomFlywheelComponents createBottomFlywheelComponents(String logPath) {
		return switch (Robot.ROBOT_TYPE) {
			case REAL -> RealFlywheelConstants.generateBottomFlywheelComponents(logPath, false);
			case SIMULATION -> null;
		};
	}

}
