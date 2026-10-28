package frc.robot.imu.factories;

import frc.robot.IDs;
import frc.robot.Robot;
import frc.robot.hardware.interfaces.IIMU;
import frc.robot.hardware.phoenix6.imu.Pigeon2IMU;
import frc.robot.imu.IMU;
import frc.robot.imu.IMUSignals;

public class IMUFactory {

	public static IMU createIMU(String logPath){
		IIMU iimu = createIIMU("IMU/");
		return new IMU(iimu, createSignals(iimu));
	}

	private static IIMU createIIMU(String logPath) {
		return switch (Robot.ROBOT_TYPE) {
			case REAL, REPLAY -> Pigeon2IMUBuilder.buildIMU(logPath, IDs.Pigeon2IDs.IMU);
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
