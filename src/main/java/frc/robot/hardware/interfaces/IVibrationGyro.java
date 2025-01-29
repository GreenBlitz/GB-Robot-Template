package frc.robot.hardware.interfaces;

import org.littletonrobotics.junction.Logger;

public interface IVibrationGyro {

	double getAngularVelocityYaw();

	double getAngularVelocityPitch();

	double getAngularVelocityRoll();

	default void logAngularVelocities(String logPath) {
		Logger.recordOutput(logPath + "Yaw", getAngularVelocityYaw());
		Logger.recordOutput(logPath + "Pitch", getAngularVelocityPitch());
		Logger.recordOutput(logPath + "Roll", getAngularVelocityRoll());
	}

	void logAngularVelocities();

}
