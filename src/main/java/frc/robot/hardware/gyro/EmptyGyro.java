package frc.robot.hardware.gyro;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.hardware.IDevice;
import frc.robot.hardware.signal.InputSignal;

public class EmptyGyro implements IGyro {

	@Override
	public boolean isConnected() {
		return false;
	}

	@Override
	public void updateSignals(InputSignal... signals) {}

	@Override
	public void setYaw(Rotation2d yaw) {}

}
