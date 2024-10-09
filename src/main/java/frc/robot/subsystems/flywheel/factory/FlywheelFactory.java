package frc.robot.subsystems.flywheel.factory;

import frc.robot.Robot;
import frc.robot.hardware.motor.sparkmax.SparkMaxDeviceID;
import frc.robot.subsystems.flywheel.Flywheel;
import frc.robot.subsystems.flywheel.FlywheelStuff;
import frc.robot.subsystems.flywheel.FlywheelConstants;

public class FlywheelFactory {

	//@formatter:off
	private static Flywheel generateRealFlywheel(
		String topLogPath,
		String bottomLogPath,
		SparkMaxDeviceID topID,
		SparkMaxDeviceID bottomID,
		Robot robot
	) {
		FlywheelStuff topStuff = RealFlywheelConstants
			.generateFlywheelStuff(topLogPath, FlywheelConstants.IS_TOP_MOTOR_INVERTED, topID);
		FlywheelStuff bottomStuff = RealFlywheelConstants
			.generateFlywheelStuff(bottomLogPath, FlywheelConstants.IS_BOTTOM_MOTOR_INVERTED, bottomID);

		return new Flywheel(topStuff, bottomStuff, FlywheelConstants.LOG_PATH, robot);
	}

	public static Flywheel create(String topLogPath, String bottomLogPath, SparkMaxDeviceID topID, SparkMaxDeviceID bottomID, Robot robot) {
		return switch (Robot.ROBOT_TYPE) {
			case REAL -> generateRealFlywheel(FlywheelConstants.LOG_PATH + topLogPath, FlywheelConstants.LOG_PATH + bottomLogPath, topID, bottomID, robot);
			case SIMULATION -> null;
		};
	}
	//@formatter:on

}

