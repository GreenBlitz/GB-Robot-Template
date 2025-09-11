package frc.robot.subsystems.swerve.factories.gyro;

import frc.robot.Robot;
import frc.robot.hardware.interfaces.IGyro;
import frc.robot.hardware.phoenix6.gyro.Pigeon2Gyro;
import frc.robot.subsystems.swerve.GyroSignals;
import frc.robot.subsystems.swerve.OdometryThread;

public class GyroFactory {

	public static IGyro createGyro(String logPath) {
		logPath += "/Gyro";
		return switch (Robot.ROBOT_TYPE) {
			case REAL -> Pigeon2GyroBuilder.buildGyro(logPath);
			case SIMULATION -> SimulationGyroBuilder.buildGyro(logPath);
		};
	}
	
	
	public static GyroSignals createThreadSignals(IGyro gyro, OdometryThread thread) {
		return switch (Robot.ROBOT_TYPE) {
			case REAL -> Pigeon2GyroBuilder.buildThreadSignals((Pigeon2Gyro) gyro, thread);
			case SIMULATION -> SimulationGyroBuilder.buildSignals();
		};
	}

}
