package frc.robot.subsystems.swerve.factories.gyro;

import frc.robot.Robot;
import frc.robot.subsystems.swerve.GyroStuff;
import frc.robot.subsystems.swerve.SwerveName;

public class GyroFactory {

	private static GyroStuff createSwerveGyroStuff(String logPath) {
		return switch (Robot.ROBOT_TYPE) {
			case REAL -> RealGyroConstants.generateGyroStuff(logPath);
			case SIMULATION -> null;// TODO
		};
	}

	public static GyroStuff create(SwerveName swerveName) {
		return switch (swerveName) {
			case SWERVE -> createSwerveGyroStuff(swerveName.getLogPath() + "Gyro/");
		};
	}

}
