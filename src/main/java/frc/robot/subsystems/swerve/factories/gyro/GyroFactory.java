package frc.robot.subsystems.swerve.factories.gyro;

import frc.robot.Robot;
import frc.robot.hardware.gyro.maple.MapleGyro;
import frc.robot.hardware.interfaces.IGyro;
import frc.robot.hardware.phoenix6.gyro.Pigeon2Gyro;
import frc.robot.subsystems.swerve.GyroSignals;
import frc.robot.subsystems.swerve.SwerveType;
import org.ironmaple.simulation.drivesims.GyroSimulation;

public class GyroFactory {

	private static IGyro createSwerveGyro(String logPath, boolean isMaple) {
		return switch (Robot.ROBOT_TYPE) {
			case REAL -> RealGyroConstants.generateGyro(logPath);
			case SIMULATION -> isMaple ? MapleGyroConstants.generateGyro(logPath) :SimulationGyroConstants.generateGyro(logPath);
		};
	}

	public static IGyro createGyro(SwerveType swerveType, boolean isMaple) {
		return switch (swerveType) {
			case SWERVE -> createSwerveGyro(swerveType.getLogPath() + "Gyro/", isMaple);
		};
	}

	private static GyroSignals createSwerveGyroSignals(IGyro gyro) {
		return switch (Robot.ROBOT_TYPE) {
			case REAL -> RealGyroConstants.generateSignals((Pigeon2Gyro) gyro);
			case SIMULATION -> gyro instanceof MapleGyro
				? MapleGyroConstants.generateSignals((MapleGyro) gyro)
				: SimulationGyroConstants.generateSignals();
		};
	}

	public static GyroSignals createSignals(SwerveType swerveType, IGyro gyro) {
		return switch (swerveType) {
			case SWERVE -> createSwerveGyroSignals(gyro);
		};
	}

}
