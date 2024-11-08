package frc.robot.hard.empties;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.hard.interfaces.IGyro;

public class EmptyGyro extends EmptyDevice implements IGyro {

	public EmptyGyro(String logPath) {
		super(logPath);
	}

	@Override
	public void setYaw(Rotation2d yaw) {}

}
