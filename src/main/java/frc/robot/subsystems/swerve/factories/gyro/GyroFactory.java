package frc.robot.subsystems.swerve.factories.gyro;

import frc.robot.Robot;
import frc.robot.subsystems.swerve.GyroStuff;
import frc.robot.subsystems.swerve.SwerveType;

public class GyroFactory {

	private static GyroStuff createSwerveGyroStuff(String logPath) {
		return switch (Robot.ROBOT_TYPE) {
			case REAL -> RealGyroConstants.generateGyroStuff(logPath);
			case SIMULATION -> null;// TODO
		};
	}

	public static GyroStuff create(SwerveType swerveType) {
		return switch (swerveType) {
			case SWERVE -> createSwerveGyroStuff(swerveType.getLogPath() + "Gyro/");
		};
	}

}
