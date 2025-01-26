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
		Logger.recordOutput(logPath + "/AccelerationX", getAccelerationX());
		Logger.recordOutput(logPath + "/AccelerationY", getAccelerationY());
		Logger.recordOutput(logPath + "/AccelerationZ", getAccelerationZ());
		Logger.recordOutput(logPath + "/AccelerationYaw", getAccelerationYaw());
		Logger.recordOutput(logPath + "/AccelerationRoll", getAccelerationRoll());
		Logger.recordOutput(logPath + "/AccelerationPitch", getAccelerationPitch());
		Logger.recordOutput(logPath + "/AccelerationMagnitude", getAccelerationMagnitude());
	}

}
