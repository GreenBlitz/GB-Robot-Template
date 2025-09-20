package frc.robot.hardware.empties;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.hardware.interfaces.Iimu;

public class EmptyIMU extends EmptyDevice implements Iimu {

	public EmptyIMU(String logPath) {
		super(logPath);
	}

	@Override
	public void setYaw(Rotation2d yaw) {}

}
