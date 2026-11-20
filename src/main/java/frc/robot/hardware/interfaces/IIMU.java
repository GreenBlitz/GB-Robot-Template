package frc.robot.hardware.interfaces;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;

public interface IIMU extends IDevice {

	void setYaw(Rotation2d yaw);

	void setOrientation(Rotation3d orientation);

}
