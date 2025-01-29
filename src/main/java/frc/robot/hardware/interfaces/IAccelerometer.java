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

	String getLogPath();

	default void logAcceleration() {
		Logger.recordOutput(getLogPath() + "X", getAccelerationX());
		Logger.recordOutput(getLogPath() + "Y", getAccelerationY());
		Logger.recordOutput(getLogPath() + "Z", getAccelerationZ());
		Logger.recordOutput(getLogPath() + "Yaw", getAccelerationYaw());
		Logger.recordOutput(getLogPath() + "Roll", getAccelerationRoll());
		Logger.recordOutput(getLogPath() + "Pitch", getAccelerationPitch());
		Logger.recordOutput(getLogPath() + "Magnitude", getAccelerationMagnitude());
	}

}
