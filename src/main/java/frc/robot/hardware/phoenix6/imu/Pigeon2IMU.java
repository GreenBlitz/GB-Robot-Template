package frc.robot.hardware.phoenix6.imu;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.hardware.interfaces.IIMU;
import frc.robot.hardware.phoenix6.Phoenix6Device;

public class Pigeon2IMU extends Phoenix6Device implements IIMU {

	private final Pigeon2Wrapper gyro;

	public Pigeon2IMU(String logPath, Pigeon2Wrapper gyro) {
		super(logPath);
		this.gyro = gyro;
		gyro.optimizeBusUtilization();
	}

	@Override
	public void setYaw(Rotation2d yaw) {
		gyro.setYaw(yaw);
	}

	@Override
	public Pigeon2Wrapper getDevice() {
		return gyro;
	}

}
