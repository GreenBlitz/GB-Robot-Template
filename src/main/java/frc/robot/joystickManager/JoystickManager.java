package frc.robot.joystickManager;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.joysticks.JoystickPorts;
import frc.joysticks.SmartJoystick;

import static frc.joysticks.JoystickPorts.*;

public class JoystickManager {

	private SmartJoystick mainJoystick = null;
	private SmartJoystick secondJoystick = null;
	private SmartJoystick thirdJoystick = null;
	private SmartJoystick fourthJoystick = null;
	private SmartJoystick fifthJoystick = null;
	private SmartJoystick sixthJoystick = null;

	private final Robot robot;

	public JoystickManager(Robot robot) {
		SendableChooser<JoystickState> mainStateChooser = new SendableChooser<>();
		addOptions(mainStateChooser, "Main joystick", MAIN);

		SendableChooser<JoystickState> secondStateChooser = new SendableChooser<>();
		addOptions(secondStateChooser, "Second joystick", SECOND);

		SendableChooser<JoystickState> thirdStateChooser = new SendableChooser<>();
		addOptions(thirdStateChooser, "Third joystick", THIRD);

		SendableChooser<JoystickState> forthStateChooser = new SendableChooser<>();
		addOptions(forthStateChooser, "Forth joystick", FOURTH);

		SendableChooser<JoystickState> fifthStateChooser = new SendableChooser<>();
		addOptions(fifthStateChooser, "Fifth joystick", FIFTH);

		SendableChooser<JoystickState> sixthStateChooser = new SendableChooser<>();
		addOptions(sixthStateChooser, "Sixth joystick", SIXTH);

		this.robot = robot;
	}

	public void setJoystickState(JoystickPorts port, JoystickState state) {
		if (state == JoystickState.NONE && portToJoystick(port) != null) {
			portToJoystick(port).setState(JoystickState.NONE);
		} else if (portToJoystick(port) == null) {
			switch (port) {
				case MAIN:
					mainJoystick = new SmartJoystick(port, state);
					JoystickBindings.configureBindings(mainJoystick, robot);
				case SECOND:
					secondJoystick = new SmartJoystick(port, state);
					JoystickBindings.configureBindings(secondJoystick, robot);
				case THIRD:
					thirdJoystick = new SmartJoystick(port, state);
					JoystickBindings.configureBindings(thirdJoystick, robot);
				case FOURTH:
					fourthJoystick = new SmartJoystick(port, state);
					JoystickBindings.configureBindings(fourthJoystick, robot);
				case FIFTH:
					fifthJoystick = new SmartJoystick(port, state);
					JoystickBindings.configureBindings(fifthJoystick, robot);
				case SIXTH:
					sixthJoystick = new SmartJoystick(port, state);
					JoystickBindings.configureBindings(sixthJoystick, robot);
			}
		} else {
			portToJoystick(port).setState(state);
		}
	}


	public SmartJoystick getJoystick(JoystickPorts port) {
		SmartJoystick joystick = null;

		switch (port) {
			case MAIN:
				joystick = mainJoystick;
			case SECOND:
				joystick = secondJoystick;
			case THIRD:
				joystick = thirdJoystick;
			case FOURTH:
				joystick = fourthJoystick;
			case FIFTH:
				joystick = fifthJoystick;
			case SIXTH:
				joystick = sixthJoystick;
		}

		if (joystick == null) {
			setJoystickState(port, JoystickState.EMPTY);
		}
		return joystick;
	}

	private SmartJoystick portToJoystick(JoystickPorts port) {
		switch (port) {
			case MAIN:
				return mainJoystick;

			case SECOND:
				return secondJoystick;

			case THIRD:
				return thirdJoystick;

			case FOURTH:
				return fourthJoystick;

			case FIFTH:
				return fifthJoystick;

			case SIXTH:
				return sixthJoystick;
		}
		return null;
	}

	private void addOptions(SendableChooser<JoystickState> chooser, String name, JoystickPorts port) {
		chooser.setDefaultOption("NONE", JoystickState.NONE);
		for (JoystickState option : JoystickState.values()) {
			chooser.addOption(String.valueOf(option), option);
		}

		chooser.onChange((joystickState) -> setJoystickState(port, (chooser.getSelected())));
		SmartDashboard.putData(name, chooser);
	}

}
