package frc.robot.subsystems.flywheel.factory;

import frc.robot.Robot;
import frc.robot.hardware.motor.sparkmax.SparkMaxDeviceID;
import frc.robot.subsystems.flywheel.Flywheel;
import frc.robot.subsystems.flywheel.FlywheelComponents;
import frc.robot.subsystems.flywheel.FlywheelConstants;

public class FlywheelFactory {

	private static Flywheel
		generateReelFlywheel(String topLogPath, String bottomLogPath, SparkMaxDeviceID topID, SparkMaxDeviceID bottomID, Robot robot) {
		FlywheelComponents topComponents = RealFlywheelConstants
			.generateFlywheelComponent(topLogPath, FlywheelConstants.IS_TOP_MOTOR_INVERTED, topID);
		FlywheelComponents bottomComponents = RealFlywheelConstants
			.generateFlywheelComponent(bottomLogPath, FlywheelConstants.IS_BOTTOM_MOTOR_INVERTED, bottomID);

		return new Flywheel(topComponents, bottomComponents, FlywheelConstants.LOG_PATH, robot);
	}

	public static Flywheel create(String topLogPath, String bottomLogPath, SparkMaxDeviceID topID, SparkMaxDeviceID bottomID, Robot robot) {
		return switch (Robot.ROBOT_TYPE) {
			case REAL -> generateReelFlywheel(topLogPath, bottomLogPath, topID, bottomID, robot);
			case SIMULATION -> null;
		};
	}

}

