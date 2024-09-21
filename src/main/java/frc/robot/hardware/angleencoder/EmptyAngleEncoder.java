package frc.robot.hardware.angleencoder;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.hardware.EmptyDevice;

public class EmptyAngleEncoder extends EmptyDevice implements IAngleEncoder {

	public EmptyAngleEncoder(String logPath) {
		super(logPath);
	}

	@Override
	public void setPosition(Rotation2d position) {}

}
