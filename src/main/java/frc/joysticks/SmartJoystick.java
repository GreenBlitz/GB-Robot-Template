package frc.joysticks;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.utils.alerts.Alert;
import frc.utils.alerts.AlertManager;
import frc.utils.alerts.PeriodicAlert;

public class SmartJoystick {

	private static final double DEADZONE = 0.07;
	private static final double DEFAULT_THRESHOLD_FOR_AXIS_BUTTON = 0.1;
	private static final double SENSITIVE_AXIS_VALUE_POWER = 2;

	private final JoystickButton A, B, X, Y, L1, R1, START, BACK, L3, R3;
	private final POVButton POV_UP, POV_RIGHT, POV_DOWN, POV_LEFT;
	private final Joystick joystick;
	private final double deadzone;
	private final String logPath;

	private BindSet bindSet;

	public SmartJoystick(SmartJoystick joystick, BindSet bindSet) {
		this(joystick, DEADZONE, bindSet);
	}

	public SmartJoystick(SmartJoystick joystick, double deadzone, BindSet bindSet) {
		this(joystick.getPort(), deadzone, bindSet);
	}

	public SmartJoystick(int port, BindSet bindSet) {
		this(port, DEADZONE, bindSet);
	}

	private SmartJoystick(int port, double deadzone, BindSet bindSet) {
		this.deadzone = deadzone;
		this.joystick = new Joystick(port);
		this.logPath = "Joysticks/" + joystick.getPort() + "/";

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

		this.bindSet = bindSet;

		//@formatter:off
		AlertManager.addAlert(
			new PeriodicAlert(
				Alert.AlertType.ERROR,
				logPath + "DisconnectedAt",
				() -> (!isConnected() && this.bindSet != BindSet.NO_JOYSTICK)
			)
		);
		//@formatter:on
	}

	public String getLogPath() {
		return logPath;
	}

	public int getPort() {
		return joystick.getPort();
	}

	public BindSet getBindSet() {
		return bindSet;
	}

	public void setBindSet(BindSet bindSet) {
		this.bindSet = bindSet;
	}

	public boolean isConnected() {
		return joystick.isConnected();
	}

	/**
	 * Sample axis value with parabolic curve, allowing for finer control for smaller values.
	 */
	public double getSensitiveAxisValue(Axis axis) {
		return sensitiveValue(getAxisValue(axis), SENSITIVE_AXIS_VALUE_POWER);
	}

	public double getAxisValue(Axis axis) {
		return isStickAxis(axis) ? applyDeadzone(axis.getValue(joystick), deadzone) : axis.getValue(joystick);
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

	private static double sensitiveValue(double axisValue, double power) {
		return Math.pow(Math.abs(axisValue), power) * Math.signum(axisValue);
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

	private static double applyDeadzone(double power, double deadzone) {
		return MathUtil.applyDeadband(power, deadzone);
	}

	public Trigger A(BindSet bindSet) {
		return A.and(() -> this.bindSet == bindSet);
	}

	public Trigger B(BindSet bindSet) {
		return B.and(() -> this.bindSet == bindSet);
	}

	public Trigger X(BindSet bindSet) {
		return X.and(() -> this.bindSet == bindSet);
	}

	public Trigger Y(BindSet bindSet) {
		return Y.and(() -> this.bindSet == bindSet);
	}

	public Trigger L1(BindSet bindSet) {
		return L1.and(() -> this.bindSet == bindSet);
	}

	public Trigger R1(BindSet bindSet) {
		return R1.and(() -> this.bindSet == bindSet);
	}

	public Trigger START(BindSet bindSet) {
		return START.and(() -> this.bindSet == bindSet);
	}

	public Trigger BACK(BindSet bindSet) {
		return START.and(() -> this.bindSet == bindSet);
	}

	public Trigger L3(BindSet bindSet) {
		return START.and(() -> this.bindSet == bindSet);
	}

	public Trigger R3(BindSet bindSet) {
		return START.and(() -> this.bindSet == bindSet);
	}

	public Trigger POV_UP(BindSet bindSet) {
		return POV_UP.and(() -> this.bindSet == bindSet);
	}

	public Trigger POV_RIGHT(BindSet bindSet) {
		return POV_RIGHT.and(() -> this.bindSet == bindSet);
	}

	public Trigger POV_DOWN(BindSet bindSet) {
		return POV_DOWN.and(() -> this.bindSet == bindSet);
	}

	public Trigger POV_LEFT(BindSet bindSet) {
		return POV_LEFT.and(() -> this.bindSet == bindSet);
	}

	private Trigger getBindSetTrigger(BindSet bindSetRequirement) {
		return new Trigger(() -> bindSetRequirement == getBindSet());
	}

}
