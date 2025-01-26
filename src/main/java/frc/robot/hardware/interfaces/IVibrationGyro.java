package frc.robot.hardware.interfaces;

import org.littletonrobotics.junction.Logger;

public interface IVibrationGyro {

	double getAngularVelocityYaw();

	double getAngularVelocityPitch();

	double getAngularVelocityRoll();

	default void logAngularVelocities(String logPath) {
		Logger.recordOutput(logPath + "/AngularVelocity/Yaw", getAngularVelocityYaw());
		Logger.recordOutput(logPath + "/AngularVelocity/Pitch", getAngularVelocityPitch());
		Logger.recordOutput(logPath + "/AngularVelocity/Roll", getAngularVelocityRoll());
	}

	void logAngularVelocities();

}
