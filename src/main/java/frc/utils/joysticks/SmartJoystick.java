package frc.utils.joysticks;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.constants.Ports;

public class SmartJoystick {

    public final JoystickButton A, B, X, Y, L1, R1, START, BACK, L3, R3;

    public final POVButton POV_UP, POV_RIGHT, POV_DOWN, POV_LEFT;

    private final Joystick joystick;

    private final double deadzone;


    public SmartJoystick(Ports.JoystickPorts joystickPort) {
        this(joystickPort, SmartJoystickConstants.DEADZONE);
    }

    public SmartJoystick(Ports.JoystickPorts joystickPort, double deadzone) {
        this(new Joystick(joystickPort.getPort()), deadzone);
    }

    private SmartJoystick(Joystick joystick, double deadzone) {
        this.deadzone = deadzone;
        this.joystick = joystick;

        this.A = new JoystickButton(this.joystick, ButtonID.A.getId());
        this.B = new JoystickButton(this.joystick, ButtonID.B.getId());
        this.X = new JoystickButton(this.joystick, ButtonID.X.getId());
        this.Y = new JoystickButton(this.joystick, ButtonID.Y.getId());

        this.L1 = new JoystickButton(this.joystick, ButtonID.L1.getId());
        this.R1 = new JoystickButton(this.joystick, ButtonID.R1.getId());

        this.BACK = new JoystickButton(this.joystick, ButtonID.BACK.getId());
        this.START = new JoystickButton(this.joystick, ButtonID.START.getId());

        this.L3 = new JoystickButton(this.joystick, ButtonID.L3.getId());
        this.R3 = new JoystickButton(this.joystick, ButtonID.R3.getId());

        this.POV_UP = new POVButton(this.joystick, ButtonID.POV_UP.getId());
        this.POV_RIGHT = new POVButton(this.joystick, ButtonID.POV_RIGHT.getId());
        this.POV_DOWN = new POVButton(this.joystick, ButtonID.POV_DOWN.getId());
        this.POV_LEFT = new POVButton(this.joystick, ButtonID.POV_LEFT.getId());
    }

    /**
     * @param power the power to rumble the joystick between [-1, 1]
     */
    public void startRumble(GenericHID.RumbleType rumbleSide, double power) {
        joystick.setRumble(rumbleSide, power);
    }


    /**
     * This function returns the value of the axis, if the stick was square instead of circle
     *
     * @param axis The axis we want to use.
     * @return stick value if stick was square.
     * <p>
     * if @ marks the 1 point of each axis:
     * <p>
     * @formatter:off
     *             Before:                                    After:
     *              (1,0)
     *           *****@*******      @ (1,1)                *************
     *      *****           *****                    *****    (1,0)     *****
     *     ***                   ***                ***  -------@-------@   ***
     *    **                       **              **   |          (1,1)|    **
     *   **                         **            **    |               |     **
     *   **                         *@ (0,1)      **    |          (0,1)@     **
     *   **                         **            **    |               |     **
     *   **                         **            **    |               |     **
     *    **                       **              **   |_______________|    **
     *     ***                   ***                ***                    ***
     *       *****           *****                    *****            *****
     *           *************                            *************
     * @formatter:on
     * </p>
     */
    public double getSquaredAxis(Axis axis) {
        if (!isStickAxis(axis)) {
            return axis.getValue(joystick);
        }
        double squaredAxisValue = getAxisValue(axis) * SmartJoystickConstants.JOYSTICK_AXIS_TO_SQUARE_FACTOR;
        return MathUtil.clamp(squaredAxisValue, -1, 1);
    }

    /**
     * Make the stick value be parabolic instead of linear. By that it gives easier and soft control in low values.
     *
     * @param axis - axis the take value from
     * @return the soft value
     */
    public double getSensitiveJoystickValue(Axis axis) {
        return getSensitiveJoystickValue(getAxisValue(axis), SmartJoystickConstants.SENSITIVE_AXIS_VALUE_POWER);
    }

    public double getSquaredSensitiveAxis(Axis axis) {
        double squaredValue = getSquaredAxis(axis);
        return getSensitiveJoystickValue(squaredValue, SmartJoystickConstants.SENSITIVE_SQUARED_AXIS_VALUE_POWER);
    }

    private static double getSensitiveJoystickValue(double axisValue, double power) {
        return Math.pow(Math.abs(axisValue), power) * Math.signum(axisValue);
    }

    private static double deadzone(double power, double deadzone) {
        return MathUtil.applyDeadband(power, deadzone);
    }

    public double getAxisValue(Axis axis) {
        return isStickAxis(axis) ? deadzone(axis.getValue(joystick), deadzone) : axis.getValue(joystick);
    }

    public AxisButton getAxisAsButton(Axis axis) {
        return getAxisAsButton(axis, SmartJoystickConstants.DEFAULT_THRESHOLD_FOR_AXIS_BUTTON);
    }

    public AxisButton getAxisAsButton(Axis axis, double threshold) {
        return axis.getAsButton(joystick, threshold);
    }

    private static boolean isStickAxis(Axis axis) {
        return (axis != Axis.LEFT_TRIGGER) && (axis != Axis.RIGHT_TRIGGER);
    }


    public enum Axis {
        LEFT_X(0, true),
        LEFT_Y(1, true),
        LEFT_TRIGGER(2, false),
        RIGHT_TRIGGER(3, false),
        RIGHT_X(4, false),
        RIGHT_Y(5, true);

        private final int axis;
        private final int invertedSign;

        Axis(int axis, boolean isInverted) {
            this.axis = axis;
            this.invertedSign = isInverted ? -1 : 1;
        }

        private AxisButton getAsButton(Joystick joystick, double threshold) {
            return new AxisButton(joystick, axis, threshold);
        }

        private double getValue(Joystick joystick) {
            return invertedSign * joystick.getRawAxis(axis);
        }
    }

}
