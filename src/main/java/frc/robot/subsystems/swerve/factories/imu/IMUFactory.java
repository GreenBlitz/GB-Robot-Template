package frc.robot.subsystems.swerve.factories.imu;

import frc.robot.Robot;
import frc.robot.hardware.interfaces.IIMU;
import frc.robot.hardware.phoenix6.imu.Pigeon2IMU;
import frc.robot.subsystems.swerve.IMUSignals;
import frc.robot.subsystems.swerve.maplewrappers.MapleGyro;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;

public class IMUFactory {

	public static IIMU createIMU(String logPath, SwerveDriveSimulation swerveDriveSimulation) {
		logPath += "/IMU";
		return switch (Robot.ROBOT_TYPE) {
			case REAL, REPLAY -> Pigeon2IMUBuilder.buildIMU(logPath);
			case SIMULATION ->
				swerveDriveSimulation == null ? EmptyIMUBuilder.buildIMU(logPath) : MapleIMUBuilder.buildIMU(logPath, swerveDriveSimulation);
		};
	}

	public static IIMU createIMU(String logPath) {
		return createIMU(logPath, null);
	}

	public static IMUSignals createSignals(IIMU gyro) {
		return switch (Robot.ROBOT_TYPE) {
			case REAL, REPLAY -> Pigeon2IMUBuilder.buildSignals((Pigeon2IMU) gyro);
			case SIMULATION -> gyro instanceof MapleGyro ? MapleIMUBuilder.buildSignals((MapleGyro) gyro) : EmptyIMUBuilder.buildSignals();
		};
	}

}
