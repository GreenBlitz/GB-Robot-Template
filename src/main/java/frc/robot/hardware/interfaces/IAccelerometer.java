package frc.robot.hardware.interfaces;

import edu.wpi.first.math.Vector;
import edu.wpi.first.math.numbers.N3;
import org.ejml.simple.SimpleMatrix;
import org.littletonrobotics.junction.Logger;

public interface IAccelerometer {

	default double getAccelerationMagnitude() {
		Vector<N3> accelerationVector = new Vector<>(
			new SimpleMatrix(new double[][] {{getAccelerationX(), getAccelerationY(), getAccelerationZ()}})
		);
		return accelerationVector.norm();
	};

	double getAccelerationX();

	double getAccelerationY();

	double getAccelerationZ();

	default double getAccelerationYaw() {
		return Math.atan2(getAccelerationY(), getAccelerationX());
	};

	default double getAccelerationPitch() {
		return Math.atan2(getAccelerationZ(), getAccelerationY());
	};

	default double getAccelerationRoll() {
		return Math.atan2(getAccelerationY(), getAccelerationZ());
	};

	void logAcceleration();

	default void logAcceleration(String logPath) {
		Logger.recordOutput(logPath + "/Acceleration/X", getAccelerationX());
		Logger.recordOutput(logPath + "/Acceleration/Y", getAccelerationY());
		Logger.recordOutput(logPath + "/Acceleration/Z", getAccelerationZ());
		Logger.recordOutput(logPath + "/Acceleration/Yaw", getAccelerationYaw());
		Logger.recordOutput(logPath + "/Acceleration/Roll", getAccelerationRoll());
		Logger.recordOutput(logPath + "/Acceleration/Pitch", getAccelerationPitch());
		Logger.recordOutput(logPath + "/Acceleration/Magnitude", getAccelerationMagnitude());
	}

}
