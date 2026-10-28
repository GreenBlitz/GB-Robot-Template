package frc.robot.hardware.empties;

import frc.utils.costomUnits.Angle2d;
import frc.robot.hardware.interfaces.IIMU;

public class EmptyIMU extends EmptyDevice implements IIMU {

	public EmptyIMU(String logPath) {
		super(logPath);
	}

	@Override
	public void setYaw(Angle2d yaw) {}

}
