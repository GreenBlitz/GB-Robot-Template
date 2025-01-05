package frc.robot.hardware.interfaces;

public interface IAccelerometer {

	double getAccelerationMagnitude();

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
