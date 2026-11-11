package frc.robot.hardware.empties;

import frc.robot.hardware.interfaces.IAngleEncoder;
import frc.utils.angle2d.Angle2d;

public class EmptyAngleEncoder extends EmptyDevice implements IAngleEncoder {

  public EmptyAngleEncoder(String logPath) {
    super(logPath);
  }

  @Override
  public void setPosition(Angle2d position) {}
}
