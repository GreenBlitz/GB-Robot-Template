package frc.robot.subsystems.swerve.factories.gyro;

import frc.robot.Robot;
import frc.robot.hardware.interfaces.IGyro;
import frc.robot.hardware.phoenix6.gyro.Pigeon2Gyro;
import frc.robot.subsystems.swerve.GyroSignals;

public class GyroFactory {

	public static IGyro createGyro(String logPath) {
		logPath += "Gyro/";
		return switch (Robot.ROBOT_TYPE) {
			case REAL -> RealGyroConstants.generateGyro(logPath);
			case SIMULATION -> SimulationGyroConstants.generateGyro(logPath);
		};
	}

	public static GyroSignals createSignals(IGyro gyro) {
		return switch (Robot.ROBOT_TYPE) {
			case REAL -> RealGyroConstants.generateSignals((Pigeon2Gyro) gyro);
			case SIMULATION -> SimulationGyroConstants.generateSignals();
		};
	}

}
