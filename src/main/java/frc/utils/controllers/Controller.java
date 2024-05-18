package frc.utils.controllers;

import frc.utils.controllers.joysticks.SmartJoystick;

public interface Controller {

    double getAxisValue(Axis axis);

    enum Axis {
        LEFT_X(0, true),
        LEFT_Y(1, true),
        LEFT_TRIGGER(2, false),
        RIGHT_TRIGGER(3, false),
        RIGHT_X(4, false),
        RIGHT_Y(5, true);

        private final int axis;

        private int inverted;

        Axis(int axis, boolean isInverted) {
            this.axis = axis;
            setInverted(isInverted);
        }

        public void setInverted(boolean isInverted) {
            inverted = isInverted ? -1 : 1;
        }

        public double getValue(SmartJoystick stick) {
            return inverted * stick.getRawAxis(axis);
        }
    }
}
