package frc.robot.hardware.phoenix6.imu;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.hardware.interfaces.IIMU;
import frc.robot.hardware.phoenix6.Phoenix6Device;

public class Pigeon2IMU extends Phoenix6Device implements IIMU {

	private final Pigeon2Wrapper imu;

	public Pigeon2IMU(String logPath, Pigeon2Wrapper imu) {
		super(logPath);
		this.imu = imu;
		imu.optimizeBusUtilization();
	}

	@Override
	public void setYaw(Rotation2d yaw) {
		imu.setYaw(yaw);
	}

	@Override
	public Pigeon2Wrapper getDevice() {
		return imu;
	}

}
