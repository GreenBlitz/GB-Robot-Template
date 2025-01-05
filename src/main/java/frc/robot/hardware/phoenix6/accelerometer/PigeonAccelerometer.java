package frc.robot.hardware.phoenix6.accelerometer;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.hardware.interfaces.IGyro;
import frc.robot.hardware.interfaces.InputSignal;
import frc.robot.hardware.phoenix6.Pigeon2Wrapper;

public class PigeonAccelerometer implements IGyro {

	public PigeonAccelerometer(String logPath, Pigeon2Wrapper accelerometer) {

	}

	@Override
	public void setYaw(Rotation2d yaw) {

	}

	@Override
	public boolean isConnected() {
		return false;
	}

	@Override
	public void updateInputs(InputSignal<?>... inputSignals) {

	}
}
