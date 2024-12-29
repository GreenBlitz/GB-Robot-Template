package frc.robot.joystickManager;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.utils.joysticks.JoystickPorts;
import frc.utils.joysticks.SmartJoystick;

import static frc.utils.joysticks.JoystickPorts.MAIN;
import static frc.utils.joysticks.JoystickPorts.THIRD;

public class JoystickManager {

	private SmartJoystick mainJoystick = null;
	private SmartJoystick secondJoystick = null;
	private SmartJoystick thirdJoystick = null;
	private SmartJoystick fourthJoystick = null;
	private SmartJoystick fifthJoystick = null;
	private SmartJoystick sixthJoystick = null;

	private SendableChooser<JoystickState> mainStateChooser;
	private SendableChooser<JoystickState> secondStateChooser;
	private SendableChooser<JoystickState> thirdStateChooser;
	private SendableChooser<JoystickState> forthStateChooser;
	private SendableChooser<JoystickState> fifthStateChooser;
	private SendableChooser<JoystickState> sixthStateChooser;


	public JoystickManager() {
		mainStateChooser = new SendableChooser<>();
		addOptions(mainStateChooser, "Main joystick", mainJoystick);

		secondStateChooser = new SendableChooser<>();
		addOptions(secondStateChooser, "Second joystick", secondJoystick);

		thirdStateChooser = new SendableChooser<>();
		addOptions(thirdStateChooser, "Third joystick", thirdJoystick);

		forthStateChooser = new SendableChooser<>();
		addOptions(forthStateChooser, "Forth joystick", fourthJoystick);

		fifthStateChooser = new SendableChooser<>();
		addOptions(fifthStateChooser, "Fifth joystick", fifthJoystick);

		sixthStateChooser = new SendableChooser<>();
		addOptions(sixthStateChooser, "Sixth joystick", sixthJoystick);
	}

	public void setJoystickState(SmartJoystick joystick, JoystickState state) {
		if (state == JoystickState.NONE && joystick != null) {
			joystick.setState(JoystickState.NONE);
		} else if (joystick == null) {

			switch (joystick.getPort()){
				case MAIN:
					mainJoystick = new SmartJoystick(joystick.getPort(), state);
				case SECOND:
					secondJoystick = new SmartJoystick(joystick.getPort(), state);
				case THIRD:
					thirdJoystick = new SmartJoystick(joystick.getPort(), state);
				case FOURTH:
					fourthJoystick = new SmartJoystick(joystick.getPort(), state);
				case FIFTH:
					fifthJoystick = new SmartJoystick(joystick.getPort(), state);
				case SIXTH:
					sixthJoystick = new SmartJoystick(joystick.getPort(), state);
			}

		} else {
			joystick.setState(state);
		}
	}


	public SmartJoystick getJoystick(JoystickPorts port) {
		SmartJoystick joystick = null;

		switch (port){
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
			setJoystickState(joystick, JoystickState.EMPTY);
		}
		return joystick;
	}

	private void addOptions(SendableChooser<JoystickState> chooser, String name, SmartJoystick joystick) {
		chooser.setDefaultOption("NONE", JoystickState.NONE);
		for (JoystickState option : JoystickState.values()) {
			chooser.addOption(String.valueOf(option), option);
		}

		chooser.onChange(((joystickState) -> setJoystickState(joystick, (mainStateChooser.getSelected()) )));
		SmartDashboard.putData(name, chooser);
	}

}
