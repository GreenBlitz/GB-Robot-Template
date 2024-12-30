package frc.robot.joystickManager;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.joysticks.SmartJoystick;

public class JoystickManager {

	private final SmartJoystick[] joysticks;

	private final Robot robot;

	public JoystickManager(Robot robot) {
		joysticks = new SmartJoystick[6];

		int currentPort = 0;

		for (SmartJoystick joystick : joysticks) {
			SendableChooser<JoystickBindSet> bindSetChooser = new SendableChooser<>();
			addOptions(bindSetChooser, currentPort);
			currentPort++;
		}

		this.robot = robot;
	}

	public void setJoystickBindSet(int port, JoystickBindSet joystickBindSet) {
		if (joysticks[port] != null) {
			joysticks[port].setBindSet(joystickBindSet);
		} else if (joysticks[port] == null) {
			joysticks[port] = new SmartJoystick(port, joystickBindSet);
			JoystickBindings.configureBindings(joysticks[port], robot);
		}
	}

	private void addOptions(SendableChooser<JoystickBindSet> chooser, int port) {
		chooser.setDefaultOption("NONE", JoystickBindSet.NONE);
		for (JoystickBindSet option : JoystickBindSet.values()) {
			chooser.addOption(String.valueOf(option), option);
		}

		chooser.onChange((joystickBindSet) -> setJoystickBindSet(port, (chooser.getSelected())));
		SmartDashboard.putData(port + " joystick", chooser);
	}

}
