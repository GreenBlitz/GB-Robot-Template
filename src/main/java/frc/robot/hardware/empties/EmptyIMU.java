package frc.robot.hardware.empties;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import frc.robot.hardware.interfaces.IIMU;

public class EmptyIMU extends EmptyDevice implements IIMU {

	public EmptyIMU(String logPath) {
		super(logPath);
	}

	@Override
	public void setYaw(Rotation2d yaw) {}

	@Override
	public void setOrientation(Rotation3d orientation) {}

}
