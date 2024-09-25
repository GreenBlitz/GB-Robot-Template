package frc.robot.subsystems.swerve.factories.gyro;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Robot;
import frc.robot.hardware.gyro.EmptyGyro;
import frc.robot.hardware.signal.EmptyAngleSignal;
import frc.robot.subsystems.swerve.GyroStuff;
import frc.robot.subsystems.swerve.SwerveName;

public class GyroFactory {

	private static GyroStuff createSwerveGyroStuff(String logPath) {
		return switch (Robot.ROBOT_TYPE) {
			case REAL -> RealGyroConstants.generateGyroStuff(logPath);
			case SIMULATION -> new GyroStuff(new EmptyGyro(logPath), new EmptyAngleSignal("yaw", new Rotation2d()));
		};
	}

	public static GyroStuff create(SwerveName swerveName) {
		return switch (swerveName) {
			case SWERVE -> createSwerveGyroStuff(swerveName.getLogPath() + "Gyro/");
		};
	}

}
