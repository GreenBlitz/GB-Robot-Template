package frc.robot.subsystems.swerve.factories.gyro;

import frc.robot.Robot;
import frc.robot.subsystems.swerve.GyroSignals;
import frc.robot.subsystems.swerve.SwerveType;

public class GyroFactory {

	private static GyroSignals createSwerveGyroStuff(String logPath) {
		return switch (Robot.ROBOT_TYPE) {
			case REAL -> RealGyroConstants.generateGyroStuff(logPath);
			case SIMULATION -> null;// TODO
		};
	}

	public static GyroSignals create(SwerveType swerveType) {
		return switch (swerveType) {
			case SWERVE -> createSwerveGyroStuff(swerveType.getLogPath() + "Gyro/");
		};
	}

}
