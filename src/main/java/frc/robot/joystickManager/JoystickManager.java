package frc.robot.joystickManager;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.joysticks.SmartJoystick;

public class JoystickManager {

	private SmartJoystick[] joysticks;

	private final Robot robot;

	public JoystickManager(Robot robot) {
		joysticks = new SmartJoystick[6];

		int currentPort = 0;

		for (SmartJoystick joystick : joysticks) {
			SendableChooser<JoystickState> stateChooser = new SendableChooser<>();
			addOptions(stateChooser, currentPort);
			currentPort++;
		}

		this.robot = robot;
	}

	public void setJoystickState(int port, JoystickState joystickState) {
		if (joysticks[port] != null) {
			joysticks[port].setState(joystickState);
		} else if (joysticks[port] == null) {
			joysticks[port] = new SmartJoystick(port, joystickState);
			JoystickBindings.configureBindings(joysticks[port], robot);
		}
	}

	private void addOptions(SendableChooser<JoystickState> chooser, int port) {
		chooser.setDefaultOption("NONE", JoystickState.NONE);
		for (JoystickState option : JoystickState.values()) {
			chooser.addOption(String.valueOf(option), option);
		}

		chooser.onChange((joystickState) -> setJoystickState(port, (chooser.getSelected())));
		SmartDashboard.putData(port + " joystick", chooser);
	}

}
