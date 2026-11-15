package frc.robot.hardware.empties;

import frc.robot.hardware.interfaces.IAngleEncoder;
import org.wpilib.math.geometry.Rotation2d;

public class EmptyAngleEncoder extends EmptyDevice implements IAngleEncoder {

  public EmptyAngleEncoder(String logPath) {
    super(logPath);
  }

  @Override
  public void setPosition(Rotation2d position) {}
}
