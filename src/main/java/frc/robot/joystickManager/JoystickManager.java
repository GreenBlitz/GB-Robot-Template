package frc.robot.joystickManager;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.joysticks.JoystickPorts;
import frc.joysticks.SmartJoystick;

import frc.joysticks.JoystickPorts.*;

public class JoystickManager {

	private SmartJoystick[] joysticks;

	private final Robot robot;

	public JoystickManager(Robot robot) {
		joysticks = new SmartJoystick[6];

		int currentPort = 0;

		for (SmartJoystick joystick : joysticks) {
			SendableChooser<JoystickState> stateChooser = new SendableChooser<>();
			addOptions(stateChooser, intToPort(currentPort));
			currentPort++;
		}

		this.robot = robot;
	}

	public void setJoystickState(JoystickPorts port, JoystickState joystickState) {
		if (portToJoystick(port) != null) {
			portToJoystick(port).setState(joystickState);
		} else if (portToJoystick(port) == null) {
			joysticks[port.getPort()] = new SmartJoystick(port, joystickState);
			JoystickBindings.configureBindings(joysticks[port.getPort()], robot);
		}
	}

	private SmartJoystick portToJoystick(JoystickPorts port) {
		return switch (port) {
			case MAIN -> joysticks[0];
			case SECOND -> joysticks[1];
			case THIRD -> joysticks[2];
			case FOURTH -> joysticks[3];
			case FIFTH -> joysticks[4];
			case SIXTH -> joysticks[5];
		};
	}

	private JoystickPorts intToPort(int port) {
		return switch (port) {
			case 0 -> JoystickPorts.MAIN;
			case 1 -> JoystickPorts.SECOND;
			case 2 -> JoystickPorts.THIRD;
			case 3 -> JoystickPorts.FOURTH;
			case 4 -> JoystickPorts.FIFTH;
			case 5 -> JoystickPorts.SIXTH;
			default -> throw new IllegalStateException("Unexpected value: " + port);
		};
	}

	private void addOptions(SendableChooser<JoystickState> chooser, JoystickPorts port) {
		chooser.setDefaultOption("NONE", JoystickState.NONE);
		for (JoystickState option : JoystickState.values()) {
			chooser.addOption(String.valueOf(option), option);
		}

		chooser.onChange((joystickState) -> setJoystickState(port, (chooser.getSelected())));
		SmartDashboard.putData(port + " joystick", chooser);
	}

}
