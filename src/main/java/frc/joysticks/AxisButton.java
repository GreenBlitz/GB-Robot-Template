package frc.joysticks;

import org.wpilib.command2.button.Trigger;
import org.wpilib.driverstation.GenericHID;

public class AxisButton extends Trigger {

  public AxisButton(GenericHID joystick, int axis, double threshold) {
    super(() -> Math.abs(joystick.getRawAxis(axis)) >= threshold);
  }
}
