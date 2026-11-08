package frc.robot.hardware.empties;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.hardware.interfaces.IIMU;

public class EmptyIMU extends EmptyDevice implements IIMU {

	private String logPath;

	public EmptyIMU(String logPath) {
		super(logPath);
		this.logPath = logPath;
	}

	@Override
	public void setYaw(Rotation2d yaw) {}

	@Override
	public String getLogPath() {
		return logPath;
	}

}
