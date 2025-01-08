package frc.joysticks;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.JoystickBindings;
import frc.utils.alerts.Alert;

public class JoystickManager {

	private static final SmartJoystick[] joysticks = new SmartJoystick[JoystickConstants.NUMBER_OF_JOYSTICK_PORTS];

	private static void setBindSet(int port, BindSet bindSet, Robot robot) {
		if (port < 0 || port >= JoystickConstants.NUMBER_OF_JOYSTICK_PORTS) {
			new Alert(Alert.AlertType.ERROR, "you cannot create a joystick for a port that doesn't exist").report();
		} else if (joysticks[port] == null) {
			createJoystick(port, robot);
		} else {
			joysticks[port].setBindSet(bindSet);
		}
	}

	private static void createJoystick(int port, Robot robot) {
		joysticks[port] = new SmartJoystick(port, BindSet.NO_JOYSTICK);
		JoystickBindings.configureBindings(joysticks[port], robot);
	}

	private static void addOptions(SendableChooser<BindSet> chooser, int port, Robot robot) {
		chooser.setDefaultOption(BindSet.NO_JOYSTICK.name(), BindSet.NO_JOYSTICK);
		for (BindSet bindSet : BindSet.values()) {
			chooser.addOption(String.valueOf(bindSet), bindSet);
		}

		chooser.onChange((bindSet) -> setBindSet(port, (chooser.getSelected()), robot));
	}

	public static void createDashboardChoosers(Robot robot) {
		for (int i = 0; i < joysticks.length; i++) {
			SendableChooser<BindSet> bindSetChooser = new SendableChooser<>();
			addOptions(bindSetChooser, i, robot);
			SmartDashboard.putData("joystick " + i, bindSetChooser);
		}
	}

}
