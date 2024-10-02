package frc.robot.subsystems.flywheel.factory;

import frc.robot.Robot;
import frc.robot.hardware.motor.sparkmax.SparkMaxDeviceID;
import frc.robot.subsystems.flywheel.FlywheelComponents;

public class FlywheelFactory {

	public static FlywheelComponents create(String logPath, boolean isMotorInverted, SparkMaxDeviceID ID) {
		return switch (Robot.ROBOT_TYPE) {
			case REAL -> RealFlywheelConstants.generateTopFlywheelComponents(logPath, isMotorInverted, ID);
			case SIMULATION -> null;
		};
	}

}
