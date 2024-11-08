package frc.robot.hard.phoenix6.gyro;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.hard.interfaces.IGyro;
import frc.robot.hard.phoenix6.Phoenix6Device;

public class Pigeon2Gyro extends Phoenix6Device implements IGyro {

	private final Pigeon2Wrapper gyro;

	public Pigeon2Gyro(String logPath, Pigeon2Wrapper gyro) {
		super(logPath);
		this.gyro = gyro;
		gyro.optimizeBusUtilization();
	}

	@Override
	public void setYaw(Rotation2d yaw) {
		gyro.setYaw(yaw);
	}

}
