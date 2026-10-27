package frc.robot.hardware.interfaces;

import org.wpilib.math.geometry.Angle2d;

public interface IIMU extends IDevice {

	void setYaw(Angle2d yaw);

}
