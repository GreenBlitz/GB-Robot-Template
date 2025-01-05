package frc.robot.hardware.phoenix6.accelerometer;

import com.ctre.phoenix6.hardware.ParentDevice;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.hardware.interfaces.IAccelerometer;
import frc.robot.hardware.interfaces.IGyro;
import frc.robot.hardware.interfaces.InputSignal;
import frc.robot.hardware.phoenix6.Phoenix6Device;
import frc.robot.hardware.phoenix6.Pigeon2Wrapper;

public class PigeonAccelerometer extends Phoenix6Device implements IAccelerometer {

	Pigeon2Wrapper accelerometer;

	public PigeonAccelerometer(String logPath, Pigeon2Wrapper accelerometer) {
		super(logPath);
		this.accelerometer = accelerometer;
	}

	@Override
	public double getAccelerationMagnitude() {
		return accelerometer.getAccelerationMagnitude();
	}

	@Override
	public double getAccelerationX() {
		return accelerometer.getAccelerationX().getValue().magnitude();
	}

	@Override
	public double getAccelerationY() {
		return accelerometer.getAccelerationY().getValue().magnitude();
	}

	@Override
	public double getAccelerationZ() {
		return accelerometer.getAccelerationZ().getValue().magnitude();
	}

	@Override
	public ParentDevice getDevice() {
		return accelerometer;
	}

}
