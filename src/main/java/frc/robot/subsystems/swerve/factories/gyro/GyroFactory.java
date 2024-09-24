package frc.robot.subsystems.swerve.factories.gyro;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Robot;
import frc.robot.hardware.gyro.EmptyGyro;
import frc.robot.hardware.gyro.IGyro;
import frc.robot.hardware.gyro.phoenix6.Pigeon2Gyro;
import frc.robot.hardware.signal.EmptyAngleSignal;
import frc.robot.hardware.signal.InputSignal;
import frc.robot.subsystems.swerve.GyroStuff;
import frc.robot.subsystems.swerve.SwerveName;

public class GyroFactory {

	public static GyroStuff create(SwerveName swerveName){
		String logPath = swerveName.getLogPath() + "Gyro";
		return switch (Robot.ROBOT_TYPE) {
			case REAL -> RealGyroConstants.generateGyroStuff(logPath);
			case SIMULATION -> new GyroStuff(new EmptyGyro(logPath), new EmptyAngleSignal("yaw", new Rotation2d()));
		};
	}

}
