package frc.joysticks;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import frc.robot.Robot;
import frc.utils.alerts.Alert;
import frc.utils.alerts.AlertManager;
import frc.utils.alerts.PeriodicAlert;
import frc.utils.math.ToleranceMath;

public class GenericSmartJoystick {

	static final double DEADZONE = 0.07;
	private static final double DEFAULT_THRESHOLD_FOR_AXIS_BUTTON = 0.1;
	private static final double SENSITIVE_AXIS_VALUE_POWER = 2;

	private final GenericHID joystick;
	private final double deadzone;
	private final String logPath;

	public GenericSmartJoystick(JoystickPorts joystickPort) {
		this(joystickPort, DEADZONE);
	}

	public GenericSmartJoystick(JoystickPorts joystickPort, double deadzone) {
		this(new Joystick(joystickPort.getPort()), deadzone, false);
	}

	public GenericSmartJoystick(JoystickPorts joystickPorts, boolean alertOnDisconnect) {
		this(new Joystick(joystickPorts.getPort()), DEADZONE, alertOnDisconnect);
	}

	public GenericSmartJoystick(JoystickPorts joystickPorts, double deadzone, boolean alertOnDisconnect) {
		this(new Joystick(joystickPorts.getPort()), deadzone, alertOnDisconnect);
	}

	public GenericSmartJoystick(GenericHID joystick, double deadzone, boolean alertOnDisconnect) {
		this.deadzone = deadzone;
		this.joystick = joystick;
		this.logPath = "Joysticks/" + joystick.getPort();

		if (Robot.ROBOT_TYPE.isReal() && alertOnDisconnect) {
			AlertManager.addAlert(new PeriodicAlert(Alert.AlertType.ERROR, logPath + "/DisconnectedAt", () -> !isConnected(), true));
		}
	}

	public String getLogPath() {
		return logPath;
	}

	public boolean isConnected() {
		return joystick.isConnected();
	}

	/**
	 * @param power the power to rumble the joystick between [-1, 1]
	 */
	public void setRumble(GenericHID.RumbleType rumbleSide, double power) {
		joystick.setRumble(rumbleSide, power);
	}

	public void stopRumble(GenericHID.RumbleType rumbleSide) {
		setRumble(rumbleSide, 0);
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
		return ToleranceMath.applyDeadband(power, deadzone);
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

