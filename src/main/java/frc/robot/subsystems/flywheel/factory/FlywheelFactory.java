package frc.robot.subsystems.flywheel.factory;

import frc.robot.Robot;
import frc.robot.hardware.motor.sparkmax.SparkMaxDeviceID;
import frc.robot.subsystems.flywheel.FlywheelComponents;

public class FlywheelFactory {

	public static FlywheelComponents create(String logPath, boolean isInverted, SparkMaxDeviceID deviceID) {
		return switch (Robot.ROBOT_TYPE) {
			case REAL -> RealFlywheelConstants.generateFlywheelComponents(logPath, isInverted, deviceID);
			case SIMULATION -> null;
		};
	}

}
