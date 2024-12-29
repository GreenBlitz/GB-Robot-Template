package frc.robot.joystickManager;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.joysticks.JoystickPorts;
import frc.joysticks.SmartJoystick;

import static frc.joysticks.JoystickPorts.*;

public class JoystickManager {

	private SmartJoystick mainJoystick;
	private SmartJoystick secondJoystick;
	private SmartJoystick thirdJoystick;
	private SmartJoystick fourthJoystick;
	private SmartJoystick fifthJoystick;
	private SmartJoystick sixthJoystick;

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

	public void setJoystickState(JoystickPorts port, JoystickState joystickState) {
		if (joystickState == JoystickState.NONE && portToJoystick(port) != null) {
			portToJoystick(port).setState(joystickState);
		}
		else if (portToJoystick(port) == null) {
			switch (port) {
				case MAIN:
					mainJoystick = new SmartJoystick(port, joystickState);
					JoystickBindings.configureBindings(mainJoystick, robot);
					break;
				case SECOND:
					secondJoystick = new SmartJoystick(port, joystickState);
					JoystickBindings.configureBindings(secondJoystick, robot);
					break;
				case THIRD:
					thirdJoystick = new SmartJoystick(port, joystickState);
					JoystickBindings.configureBindings(thirdJoystick, robot);
					break;
				case FOURTH:
					fourthJoystick = new SmartJoystick(port, joystickState);
					JoystickBindings.configureBindings(fourthJoystick, robot);
					break;
				case FIFTH:
					fifthJoystick = new SmartJoystick(port, joystickState);
					JoystickBindings.configureBindings(fifthJoystick, robot);
					break;
				case SIXTH:
					sixthJoystick = new SmartJoystick(port, joystickState);
					JoystickBindings.configureBindings(sixthJoystick, robot);
					break;
			}

		}
		else {
			portToJoystick(port).setState(joystickState);
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
        return switch (port) {
            case MAIN -> mainJoystick;
            case SECOND -> secondJoystick;
            case THIRD -> thirdJoystick;
            case FOURTH -> fourthJoystick;
            case FIFTH -> fifthJoystick;
            case SIXTH -> sixthJoystick;
        };
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
