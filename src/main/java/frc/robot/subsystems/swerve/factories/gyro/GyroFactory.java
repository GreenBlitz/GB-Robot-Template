package frc.robot.subsystems.swerve.factories.gyro;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Robot;
import frc.robot.hardware.gyro.EmptyGyro;
import frc.robot.hardware.gyro.IGyro;
import frc.robot.hardware.gyro.phoenix6.Pigeon2Gyro;
import frc.robot.hardware.signal.InputSignal;
import frc.robot.subsystems.swerve.SwerveName;

public class GyroFactory {

	public static IGyro create(SwerveName swerveName) {
		return switch (Robot.ROBOT_TYPE) {
			case REAL -> new Pigeon2Gyro(swerveName.getLogPath() + "Gyro", RealGyroConstants.getPigeon2Wrapper());
			case SIMULATION -> new EmptyGyro(swerveName.getLogPath() + "Gyro");
		};
	}

	public static InputSignal<Rotation2d> createYawSignal() {
		return switch (Robot.ROBOT_TYPE) {
			case REAL -> RealGyroConstants.getYawSignal();
			case SIMULATION -> null;
		};
	}

}
