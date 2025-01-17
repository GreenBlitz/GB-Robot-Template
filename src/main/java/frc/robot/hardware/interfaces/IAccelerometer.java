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
		Logger.recordOutput(logPath + "accelerationX", getAccelerationX());
		Logger.recordOutput(logPath + "accelerationY", getAccelerationY());
		Logger.recordOutput(logPath + "accelerationZ", getAccelerationZ());
		Logger.recordOutput(logPath + "accelerationYaw", getAccelerationYaw());
		Logger.recordOutput(logPath + "accelerationRoll", getAccelerationRoll());
		Logger.recordOutput(logPath + "accelerationPitch", getAccelerationPitch());
		Logger.recordOutput(logPath + "accelerationMagnitude", getAccelerationMagnitude());
	}

}
