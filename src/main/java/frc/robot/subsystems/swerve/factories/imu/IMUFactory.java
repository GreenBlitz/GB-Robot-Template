package frc.robot.subsystems.swerve.factories.imu;

import frc.robot.Robot;
import frc.robot.hardware.interfaces.IIMU;
import frc.robot.subsystems.swerve.IMUSignals;

public class IMUFactory {

	public static IIMU createIMU(String logPath) {
		logPath += "/IMU";
		return switch (Robot.ROBOT_TYPE) {
			case REAL -> null;
			case SIMULATION -> SimulationIMUBuilder.buildIMU(logPath);
			case REPLAY -> null;
		};
	}

	public static IMUSignals createSignals(IIMU gyro) {
		return switch (Robot.ROBOT_TYPE) {
			case REAL -> null;
			case SIMULATION -> SimulationIMUBuilder.buildSignals();
			case REPLAY -> null;
		};
	}

}
