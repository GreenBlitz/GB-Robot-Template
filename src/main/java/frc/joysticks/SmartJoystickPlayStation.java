package frc.joysticks;

import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;


public class SmartJoystickPlayStation extends GenericSmartJoystick {

	public JoystickButton XButton, circleButton, squareButton, triangularButton, L1, R1, L3, R3, optionButton, shareButton;
	public final POVButton POV_UP, POV_RIGHT, POV_DOWN, POV_LEFT;
	private final PS4Controller joystick;

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

	public SmartJoystickPlayStation(PS4Controller joystick, double deadzone, boolean alertOnDisconnect) {
		super(joystick, deadzone, alertOnDisconnect);
		this.joystick = joystick;

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
	}

}
