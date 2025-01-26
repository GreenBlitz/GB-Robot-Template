package frc.robot.hardware.interfaces;

import org.littletonrobotics.junction.Logger;

public interface IVibrationGyro {

	double getAngularVelocityYaw();

	double getAngularVelocityPitch();

	double getAngularVelocityRoll();

	default void logAngularVelocities(String logPath) {
		Logger.recordOutput(logPath + "AngularVelocityYaw", getAngularVelocityYaw());
		Logger.recordOutput(logPath + "AngularVelocityPitch", getAngularVelocityPitch());
		Logger.recordOutput(logPath + "AngularVelocityRoll", getAngularVelocityRoll());
	}

	void logAngularVelocities();

}
