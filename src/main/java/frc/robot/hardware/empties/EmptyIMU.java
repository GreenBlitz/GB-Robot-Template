package frc.robot.hardware.empties;

import frc.robot.hardware.interfaces.IIMU;
import frc.utils.angle2d.Angle2d;

public class EmptyIMU extends EmptyDevice implements IIMU {

  public EmptyIMU(String logPath) {
    super(logPath);
  }

  @Override
  public void setYaw(Angle2d yaw) {}
}
