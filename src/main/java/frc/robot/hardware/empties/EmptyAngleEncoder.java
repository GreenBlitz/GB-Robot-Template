package frc.robot.hardware.empties;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.hardware.interfaces.IAngleEncoder;

public class EmptyAngleEncoder extends EmptyDevice implements IAngleEncoder {

	public EmptyAngleEncoder(String logPath) {
		super(logPath);
	}

	@Override
	public void setPosition(Rotation2d position) {}

}
