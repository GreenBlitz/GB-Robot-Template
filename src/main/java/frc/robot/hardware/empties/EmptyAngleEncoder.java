package frc.robot.hardware.empties;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.hardware.interfaces.IAngleEncoder;

public class EmptyAngleEncoder extends EmptyDevice implements IAngleEncoder {

	private final String logPath;

	public EmptyAngleEncoder(String logPath) {
		super(logPath);
		this.logPath = logPath;
	}

	@Override
	public void setPosition(Rotation2d position) {}

	@Override
	public String getLogPath() {
		return logPath;
	}

}
