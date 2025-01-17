package frc.robot.hardware.phoenix6.pigeon;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.hardware.interfaces.IAccelerometer;
import frc.robot.hardware.interfaces.IGyro;
import frc.robot.hardware.phoenix6.Phoenix6Device;
import frc.robot.hardware.phoenix6.Pigeon2Wrapper;

public class PigeonHandler extends Phoenix6Device implements IGyro, IAccelerometer {

	private final Pigeon2Wrapper pigeon;
	String logPath;

	public PigeonHandler(String logPath, Pigeon2Wrapper pigeon) {
		super(logPath);
		this.pigeon = pigeon;
		this.logPath = logPath;
		pigeon.optimizeBusUtilization();
	}

	@Override
	public void setYaw(Rotation2d yaw) {
		pigeon.setYaw(yaw);
	}

	@Override
	public Pigeon2Wrapper getDevice() {
		return pigeon;
	}

	@Override
	public double getAccelerationX() {
		return pigeon.getAccelerationX().getValue().magnitude();
	}

	@Override
	public double getAccelerationY() {
		return pigeon.getAccelerationY().getValue().magnitude();
	}

	@Override
	public double getAccelerationZ() {
		return pigeon.getAccelerationZ().getValue().magnitude();
	}

	@Override
	public void log() {
		log(logPath);
	}

}
