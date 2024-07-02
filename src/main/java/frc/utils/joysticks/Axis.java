package frc.utils.joysticks;

import edu.wpi.first.wpilibj.Joystick;

public enum Axis {

    LEFT_X(0, true),
    LEFT_Y(1, true),
    LEFT_TRIGGER(2, false),
    RIGHT_TRIGGER(3, false),
    RIGHT_X(4, false),
    RIGHT_Y(5, true);

    private final int id;
    private final int invertedSign;

    Axis(int id, boolean isInverted) {
        this.id = id;
        this.invertedSign = isInverted ? -1 : 1;
    }

    AxisButton getAsButton(Joystick joystick, double threshold) {
        return new AxisButton(joystick, id, threshold);
    }

    double getValue(Joystick joystick) {
        return invertedSign * joystick.getRawAxis(id);
    }

}
