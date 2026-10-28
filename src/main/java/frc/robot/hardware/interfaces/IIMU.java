package frc.robot.hardware.interfaces;

import org.wpilib.math.geometry.Rotation2d;

public interface IIMU extends IDevice {

	void setYaw(Rotation2d yaw);

}
