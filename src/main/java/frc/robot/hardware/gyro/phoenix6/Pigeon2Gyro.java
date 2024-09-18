package frc.robot.hardware.gyro.phoenix6;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.hardware.CTREDevice;
import frc.robot.hardware.gyro.IGyro;

public class Pigeon2Gyro extends CTREDevice implements IGyro {

	private final Pigeon2Wrapper gyro;

	public Pigeon2Gyro(String logPath, Pigeon2Wrapper gyro) {
		super(logPath);
		this.gyro = gyro;
	}

	@Override
	public void setYaw(Rotation2d yaw) {
		gyro.setYaw(yaw);
	}

}
