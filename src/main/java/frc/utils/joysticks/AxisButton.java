package frc.utils.joysticks;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class AxisButton extends Trigger {

    public AxisButton(GenericHID joystick, int axis, double threshold) {
        super(() -> Math.abs(joystick.getRawAxis(axis)) >= threshold);
    }

}
