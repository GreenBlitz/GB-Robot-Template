package frc.robot.imu.factories;

import frc.robot.IDs;
import frc.robot.Robot;
import frc.robot.hardware.interfaces.IIMU;
import frc.robot.hardware.phoenix6.imu.Pigeon2IMU;
import frc.robot.imu.IMUWrapper;
import frc.robot.imu.IMUSignals;

public class IMUFactory {

	public static IMUWrapper createIMU(String logPath) {
		IIMU imu = createIIMU(logPath);
		return new IMUWrapper(imu, createSignals(imu));
	}

	private static IIMU createIIMU(String logPath) {
		return switch (Robot.ROBOT_TYPE) {
			case REAL, REPLAY -> Pigeon2IMUBuilder.buildIMU(logPath, IDs.Pigeon2IDs.CHASSIS);
			case SIMULATION -> SimulationIMUBuilder.buildIMU(logPath);
		};
	}

	private static IMUSignals createSignals(IIMU imu) {
		return switch (Robot.ROBOT_TYPE) {
			case REAL, REPLAY -> Pigeon2IMUBuilder.buildSignals((Pigeon2IMU) imu);
			case SIMULATION -> SimulationIMUBuilder.buildSignals();
		};
	}

}
