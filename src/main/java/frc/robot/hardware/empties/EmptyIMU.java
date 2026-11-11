package frc.robot.hardware.empties;

import frc.robot.hardware.interfaces.IIMU;
import org.wpilib.math.geometry.Rotation2d;

public class EmptyIMU extends EmptyDevice implements IIMU {

  public EmptyIMU(String logPath) {
    super(logPath);
  }

  @Override
  public void setYaw(Rotation2d yaw) {}
}
