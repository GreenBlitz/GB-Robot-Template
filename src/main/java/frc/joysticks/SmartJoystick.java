package frc.joysticks;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;

public class SmartJoystick extends GenericSmartJoystick {

	public final JoystickButton A, B, X, Y, L1, R1, START, BACK, L3, R3;
	public final POVButton POV_UP, POV_RIGHT, POV_DOWN, POV_LEFT;
	private final Joystick joystick;

	public SmartJoystick(JoystickPorts joystickPort) {
		this(joystickPort, DEADZONE);
	}

	public SmartJoystick(JoystickPorts joystickPort, double deadzone) {
		this(new Joystick(joystickPort.getPort()), deadzone, false);
	}

	public SmartJoystick(JoystickPorts joystickPorts, boolean alertOnDisconnect) {
		this(new Joystick(joystickPorts.getPort()), DEADZONE, alertOnDisconnect);
	}

	public SmartJoystick(JoystickPorts joystickPorts, double deadzone, boolean alertOnDisconnect) {
		this(new Joystick(joystickPorts.getPort()), deadzone, alertOnDisconnect);
	}

	public SmartJoystick(Joystick joystick, double deadzone, boolean alertOnDisconnect) {
		super(joystick, deadzone, alertOnDisconnect);
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

}
