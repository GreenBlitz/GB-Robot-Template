package frc.robot.hardware.interfaces;

import edu.wpi.first.math.Vector;
import edu.wpi.first.math.numbers.N3;
import org.ejml.simple.SimpleMatrix;

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
		return Math.cos(getAccelerationX()) + Math.sin(getAccelerationY());
	};

	default double getAccelerationPitch() {
		return Math.cos(getAccelerationZ()) + Math.sin(getAccelerationY());
	};

	default double getAccelerationRoll() {
		return Math.cos(getAccelerationY()) + Math.sin(getAccelerationZ());
	};

}
