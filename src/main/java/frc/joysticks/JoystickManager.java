package frc.joysticks;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.JoystickBindings;
import frc.JoystickBindings.BindSet;

public class JoystickManager {

	private static final SmartJoystick[] joysticks = new SmartJoystick[JoystickConstants.NUMBER_OF_JOYSTICK_PORTS];

	private static void setBindSet(int port, BindSet bindSet, Robot robot) {
		if (joysticks[port] == null) {
			createJoystick(port, robot);
		}
		joysticks[port].setBindSet(bindSet);
	}

	private static void createJoystick(int port, Robot robot) {
		joysticks[port] = new SmartJoystick(port, BindSet.NONE);
		JoystickBindings.configureBindings(joysticks[port], robot);
	}

	private static void addOptions(SendableChooser<BindSet> chooser, int port, Robot robot) {
		chooser.setDefaultOption("NONE", BindSet.NONE);
		for (BindSet option : BindSet.values()) {
			chooser.addOption(String.valueOf(option), option);
		}

		chooser.onChange((bindSet) -> setBindSet(port, (chooser.getSelected()), robot));
		SmartDashboard.putData(port + " joystick", chooser);
	}

	public static void createDashboardChoosers(Robot robot) {
		for (int i = 0; i < joysticks.length; i++) {
			SendableChooser<BindSet> bindSetChooser = new SendableChooser<>();
			addOptions(bindSetChooser, i, robot);
		}
	}

}
