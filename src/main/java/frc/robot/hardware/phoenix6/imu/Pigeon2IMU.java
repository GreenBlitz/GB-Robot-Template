package frc.robot.hardware.phoenix6.imu;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import frc.robot.hardware.interfaces.IIMU;
import frc.robot.hardware.phoenix6.Phoenix6Device;

public class Pigeon2IMU extends Phoenix6Device implements IIMU {

	private final Pigeon2Wrapper imu;

	public Pigeon2IMU(String logPath, Pigeon2Wrapper imu) {
		super(logPath, imu.getBusChain());
		this.imu = imu;
		imu.optimizeBusUtilization();
	}

	@Override
	public void setYaw(Rotation2d yaw) {
		imu.setYaw(yaw);
	}

	@Override
	public void setOrientation(Rotation3d orientation) {
		imu.setRoll(Rotation2d.fromRadians(orientation.getX()));
		imu.setPitch(Rotation2d.fromRadians(orientation.getY()));
		setYaw(Rotation2d.fromRadians(orientation.getZ()));
	}

	@Override
	public Pigeon2Wrapper getDevice() {
		return imu;
	}

}
