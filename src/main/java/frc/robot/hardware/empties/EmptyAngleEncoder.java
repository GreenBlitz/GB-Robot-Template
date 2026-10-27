package frc.robot.hardware.empties;

import org.wpilib.math.geometry.Angle2d;
import frc.robot.hardware.interfaces.IAngleEncoder;

public class EmptyAngleEncoder extends EmptyDevice implements IAngleEncoder {

	public EmptyAngleEncoder(String logPath) {
		super(logPath);
	}

	@Override
	public void setPosition(Angle2d position) {}

}
