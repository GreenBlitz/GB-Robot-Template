package frc.robot.hardware.interfaces;

import edu.wpi.first.math.geometry.Rotation2d;

public interface IIMU extends IDevice {

	void setYaw(Rotation2d yaw);

}
