package frc.robot.subsystems.swerve.factories.imu;

import frc.robot.Robot;
import frc.robot.hardware.interfaces.IIMU;
import frc.robot.subsystems.swerve.IMUSignals;

public class IMUFactory {

	public static IIMU createIMU(String logPath) {
		logPath += "/IMU";
		return switch (Robot.ROBOT_TYPE) {
			case REAL, REPLAY, SIMULATION -> SimulationIMUBuilder.buildIMU(logPath);
		};
	}

	public static IMUSignals createSignals(IIMU gyro) {
		return switch (Robot.ROBOT_TYPE) {
			case REAL, REPLAY, SIMULATION -> SimulationIMUBuilder.buildSignals();
		};
	}

}
