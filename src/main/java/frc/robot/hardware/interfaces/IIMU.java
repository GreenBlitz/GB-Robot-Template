package frc.robot.hardware.interfaces;

import frc.utils.angle2d.Angle2d;

public interface IIMU extends IDevice {

  void setYaw(Angle2d yaw);
}
