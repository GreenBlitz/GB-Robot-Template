package frc.robot.subsystems.swerve.factories.gyro;

import frc.robot.Robot;
import frc.robot.subsystems.swerve.GyroStuff;
import frc.robot.subsystems.swerve.SwerveType;
import org.ironmaple.simulation.drivesims.GyroSimulation;

public class GyroFactory {

	private static GyroStuff createSwerveGyroStuff(String logPath, GyroSimulation gyroSimulation) {
		return switch (Robot.ROBOT_TYPE) {
			case REAL -> RealGyroConstants.generateGyroStuff(logPath);
			case SIMULATION -> SimulationGyroConstants.generateGyroStuff(logPath, gyroSimulation);
		};
	}

	public static GyroStuff create(SwerveType swerveType, GyroSimulation gyroSimulation) {
		return switch (swerveType) {
			case SWERVE -> createSwerveGyroStuff(swerveType.getLogPath() + "Gyro/", gyroSimulation);
		};
	}

}
