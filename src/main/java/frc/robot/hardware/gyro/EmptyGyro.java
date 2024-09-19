package frc.robot.hardware.gyro;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.hardware.EmptyDevice;

public class EmptyGyro extends EmptyDevice implements IGyro {

	public EmptyGyro(String logPath) {
		super(logPath);
	}

	@Override
	public void setYaw(Rotation2d yaw) {}

}
