package frc.utils.joysticks;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.constants.Ports;

public class SmartJoystick {

    private static final double DEADZONE = 0.07;
    private static final double DEFAULT_THRESHOLD_FOR_AXIS_BUTTON = 0.1;
    private static final double SENSITIVE_AXIS_VALUE_POWER = 2;

    public final JoystickButton A, B, X, Y, L1, R1, START, BACK, L3, R3;
    public final POVButton POV_UP, POV_RIGHT, POV_DOWN, POV_LEFT;
    private final Joystick joystick;
    private final double deadzone;

    public SmartJoystick(Ports.JoystickPorts joystickPort) {
        this(joystickPort, DEADZONE);
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

    public void stopRumble(GenericHID.RumbleType rumbleSide) {
        joystick.setRumble(rumbleSide, 0);
    }

    /**
     * Sample axis value with parabolic curve, allowing for finer control for smaller values.
     */
    public double getSensitiveAxisValue(Axis axis) {
        return sensitiveValue(getAxisValue(axis), SENSITIVE_AXIS_VALUE_POWER);
    }

    private static double sensitiveValue(double axisValue, double power) {
        return Math.pow(Math.abs(axisValue), power) * Math.signum(axisValue);
    }

    public double getAxisValue(Axis axis) {
        return isStickAxis(axis) ? applyDeadzone(axis.getValue(joystick), deadzone) : axis.getValue(joystick);
    }

    private static double applyDeadzone(double power, double deadzone) {
        return MathUtil.applyDeadband(power, deadzone);
    }

    public AxisButton getAxisAsButton(Axis axis) {
        return getAxisAsButton(axis, DEFAULT_THRESHOLD_FOR_AXIS_BUTTON);
    }

    public AxisButton getAxisAsButton(Axis axis, double threshold) {
        return axis.getAsButton(joystick, threshold);
    }

    private static boolean isStickAxis(Axis axis) {
        return (axis != Axis.LEFT_TRIGGER) && (axis != Axis.RIGHT_TRIGGER);
    }

}
