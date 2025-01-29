package frc.robot.subsystems.swerve.factories.gyro;

import frc.robot.Robot;
import frc.robot.hardware.interfaces.IAccelerometer;
import frc.robot.hardware.interfaces.IGyro;
import frc.robot.hardware.interfaces.IVibrationGyro;
import frc.robot.hardware.mechanisms.SimulatedAccelerometer;
import frc.robot.hardware.phoenix6.pigeon.PigeonHandler;
import frc.robot.subsystems.swerve.GyroSignals;

public class PigeonFactory {

	public static IGyro createGyro(String logPath) {
		logPath += "/Gyro";
		return switch (Robot.ROBOT_TYPE) {
			case REAL -> Pigeon2GyroBuilder.buildPigeon(logPath);
			case SIMULATION -> SimulationGyroBuilder.buildGyro(logPath);
		};
	}

	public static IAccelerometer createAccelerometer(String logPath) {
		logPath += "/Accelerometer";
		return switch (Robot.ROBOT_TYPE) {
			case REAL -> Pigeon2GyroBuilder.buildPigeon(logPath);
			case SIMULATION -> new SimulatedAccelerometer(logPath, null, 0);
		};
	}

	public static IVibrationGyro createVibrationGyro(String logPath) {
		logPath += "/VibrationGyro";
		return switch (Robot.ROBOT_TYPE) {
			case REAL -> Pigeon2GyroBuilder.buildPigeon(logPath);
			case SIMULATION -> null;
		};
	}

	public static GyroSignals createSignals(IGyro gyro) {
		return switch (Robot.ROBOT_TYPE) {
			case REAL -> Pigeon2GyroBuilder.buildSignals((PigeonHandler) gyro);
			case SIMULATION -> SimulationGyroBuilder.buildSignals();
		};
	}

}
