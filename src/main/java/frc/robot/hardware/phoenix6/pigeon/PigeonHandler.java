package frc.robot.hardware.phoenix6.pigeon;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.hardware.interfaces.IAccelerometer;
import frc.robot.hardware.interfaces.IGyro;
import frc.robot.hardware.interfaces.IVibrationGyro;
import frc.robot.hardware.phoenix6.Phoenix6Device;
import frc.robot.hardware.phoenix6.Pigeon2Wrapper;

public class PigeonHandler extends Phoenix6Device implements IGyro, IAccelerometer, IVibrationGyro {

	private final Pigeon2Wrapper pigeon;
	private final String logPath;
	private final PigeonOffsets offsets;

	public PigeonHandler(String logPath, Pigeon2Wrapper pigeon, PigeonOffsets offsets) {
		super(logPath);
		this.pigeon = pigeon;
		this.logPath = logPath;
		this.offsets = offsets;
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
		return pigeon.getAccelerationX().getValue().magnitude() - offsets.XOffset();
	}

	@Override
	public double getAccelerationY() {
		return pigeon.getAccelerationY().getValue().magnitude() - offsets.YOffset();
	}

	@Override
	public double getAccelerationZ() {
		return pigeon.getAccelerationZ().getValue().magnitude() - offsets.ZOffset();
	}

	@Override
	public double getAngularVelocityYaw() {
		return pigeon.getAngularVelocityZWorld().getValue().magnitude();
	}

	@Override
	public double getAngularVelocityPitch() {
		return pigeon.getAngularVelocityXWorld().getValue().magnitude();
	}

	@Override
	public double getAngularVelocityRoll() {
		return pigeon.getAngularVelocityYWorld().getValue().magnitude();
	}

	@Override
	public void logAngularVelocities() {
		logAngularVelocities(logPath + "/AngularVelocity");
	}

	@Override
	public void logAcceleration() {
		logAcceleration(logPath + "/Acceleration");
	}

}
