package frc.robot.subsystems.swerve.factories.gyro;

import frc.robot.Robot;
import frc.robot.hardware.interfaces.IGyro;
import frc.robot.hardware.phoenix6.gyro.Pigeon2Gyro;
import frc.robot.subsystems.swerve.GyroSignals;
import frc.robot.subsystems.swerve.SwerveType;
import org.ironmaple.simulation.drivesims.GyroSimulation;

public class GyroFactory {

<<<<<<< HEAD
	private static GyroStuff createSwerveGyroStuff(String logPath, GyroSimulation gyroSimulation) {
		return switch (Robot.ROBOT_TYPE) {
			case REAL -> RealGyroConstants.generateGyroStuff(logPath);
			case SIMULATION -> SimulationGyroConstants.generateGyroStuff(logPath, gyroSimulation);
		};
	}

	public static GyroStuff create(SwerveType swerveType, GyroSimulation gyroSimulation) {
		return switch (swerveType) {
			case SWERVE -> createSwerveGyroStuff(swerveType.getLogPath() + "Gyro/", gyroSimulation);
=======
	private static IGyro createSwerveGyro(String logPath) {
		return switch (Robot.ROBOT_TYPE) {
			case REAL -> RealGyroConstants.generateGyro(logPath);
			case SIMULATION -> SimulationGyroConstants.generateGyro(logPath);
		};
	}

	public static IGyro createGyro(SwerveType swerveType) {
		return switch (swerveType) {
			case SWERVE -> createSwerveGyro(swerveType.getLogPath() + "Gyro/");
		};
	}

	private static GyroSignals createSwerveGyroSignals(IGyro gyro) {
		return switch (Robot.ROBOT_TYPE) {
			case REAL -> RealGyroConstants.generateSignals((Pigeon2Gyro) gyro);
			case SIMULATION -> SimulationGyroConstants.generateSignals();
		};
	}

	public static GyroSignals createSignals(SwerveType swerveType, IGyro gyro) {
		return switch (swerveType) {
			case SWERVE -> createSwerveGyroSignals(gyro);
>>>>>>> core-swerve
		};
	}

}
