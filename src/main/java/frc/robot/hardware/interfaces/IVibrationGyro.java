package frc.robot.hardware.interfaces;

import org.littletonrobotics.junction.Logger;

public interface IVibrationGyro {

	double getAngularVelocityYaw();

	double getAngularVelocityPitch();

	double getAngularVelocityRoll();

	String getLogPath();

	default void logAngularVelocities() {
		Logger.recordOutput(getLogPath() + "Yaw", getAngularVelocityYaw());
		Logger.recordOutput(getLogPath() + "Pitch", getAngularVelocityPitch());
		Logger.recordOutput(getLogPath() + "Roll", getAngularVelocityRoll());
	}

}
