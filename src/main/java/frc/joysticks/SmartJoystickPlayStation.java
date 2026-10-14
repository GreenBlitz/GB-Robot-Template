package frc.joysticks;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.Robot;
import frc.utils.alerts.Alert;
import frc.utils.alerts.AlertManager;
import frc.utils.alerts.PeriodicAlert;
import frc.utils.math.ToleranceMath;


public class SmartJoystickPlayStation {

	private static final double DEADZONE = 0.07;
	private static final double DEFAULT_THRESHOLD_FOR_AXIS_BUTTON = 0.1;
	private static final double SENSITIVE_AXIS_VALUE_POWER = 2;

	public JoystickButton XButton, circleButton, squareButton, triangularButton, L1, R1, L3, R3, optionButton, shareButton;
	public final POVButton POV_UP, POV_RIGHT, POV_DOWN, POV_LEFT;
	private final PS4Controller joystick;
	private final double deadzone;
	private final String logPath;

	public SmartJoystickPlayStation(JoystickPorts joystickPort) {
		this(joystickPort, DEADZONE);
	}

	public SmartJoystickPlayStation(JoystickPorts joystickPort, double deadzone) {
		this(new PS4Controller(joystickPort.getPort()), deadzone, false);
	}

	public SmartJoystickPlayStation(JoystickPorts joystickPorts, boolean alertOnDisconnect) {
		this(new PS4Controller(joystickPorts.getPort()), DEADZONE, alertOnDisconnect);
	}

	public SmartJoystickPlayStation(JoystickPorts joystickPorts, double deadzone, boolean alertOnDisconnect) {
		this(new PS4Controller(joystickPorts.getPort()), deadzone, alertOnDisconnect);
	}

	private SmartJoystickPlayStation(PS4Controller joystick, double deadzone, boolean alertOnDisconnect) {
		this.deadzone = deadzone;
		this.joystick = joystick;
		this.logPath = "Joysticks/" + joystick.getPort();

		this.L3 = new JoystickButton(this.joystick, PS4Controller.Button.kL3.value);
		this.circleButton = new JoystickButton(this.joystick, PS4Controller.Button.kCircle.value);
		this.optionButton = new JoystickButton(this.joystick, PS4Controller.Button.kOptions.value);
		this.shareButton = new JoystickButton(this.joystick, PS4Controller.Button.kShare.value);
		this.squareButton = new JoystickButton(this.joystick, PS4Controller.Button.kCross.value);
		this.L1 = new JoystickButton(this.joystick, PS4Controller.Button.kL1.value);
		this.R1 = new JoystickButton(this.joystick, PS4Controller.Button.kR1.value);
		this.triangularButton = new JoystickButton(this.joystick, PS4Controller.Button.kTriangle.value);
		this.R3 = new JoystickButton(joystick, PS4Controller.Button.kR3.value);

		this.POV_UP = new POVButton(this.joystick, 0);
		this.POV_RIGHT = new POVButton(this.joystick, 90);
		this.POV_DOWN = new POVButton(this.joystick, 180);
		this.POV_LEFT = new POVButton(this.joystick, 270);

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
		return isStickAxis(axis) ? applyDeadzone(joystick.getAxisCount(), deadzone) : joystick.getAxisCount();
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
