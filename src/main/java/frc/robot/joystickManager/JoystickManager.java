package frc.robot.joystickManager;

import com.fasterxml.jackson.databind.JsonSerializer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.utils.joysticks.JoystickPorts;
import frc.utils.joysticks.SmartJoystick;

public class JoystickManager {

	private static SmartJoystick mainJoystick = null;
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


	public JoystickManager(){

		mainStateChooser = new SendableChooser<>();
		mainStateChooser.setDefaultOption("NONE", JoystickState.NONE);
		mainStateChooser.onChange(((joystickState)->setMainJoystickState((mainStateChooser.getSelected()))));

		secondStateChooser = new SendableChooser<>();
		secondStateChooser.setDefaultOption("NONE", JoystickState.NONE);
		secondStateChooser.onChange(((joystickState)->setMainJoystickState((mainStateChooser.getSelected()))));

		thirdStateChooser = new SendableChooser<>();
		thirdStateChooser.setDefaultOption("NONE", JoystickState.NONE);
		thirdStateChooser.onChange(((joystickState)->setMainJoystickState((mainStateChooser.getSelected()))));

		forthStateChooser = new SendableChooser<>();
		forthStateChooser.setDefaultOption("NONE", JoystickState.NONE);
		forthStateChooser.onChange(((joystickState)->setMainJoystickState((mainStateChooser.getSelected()))));

		fifthStateChooser = new SendableChooser<>();
		fifthStateChooser.setDefaultOption("NONE", JoystickState.NONE);
		fifthStateChooser.onChange(((joystickState)->setMainJoystickState((mainStateChooser.getSelected()))));

		sixthStateChooser = new SendableChooser<>();
		sixthStateChooser.setDefaultOption("NONE", JoystickState.NONE);
		sixthStateChooser.onChange(((joystickState)->setMainJoystickState((mainStateChooser.getSelected()))));


		SmartDashboard.putData("joysticks/main joystick", mainStateChooser);
	}

	public static void setMainJoystickState(JoystickState state) {
		if (state == JoystickState.NONE) {
			mainJoystick = null;
		} else if (mainJoystick == null) {
			mainJoystick = new SmartJoystick(JoystickPorts.MAIN, state);
		} else {
			mainJoystick.setState(state);
		}
	}

	public void setSecondJoystickState(JoystickState state) {
		if (state == JoystickState.NONE) {
			secondJoystick = null;
		} else if (secondJoystick == null) {
			secondJoystick = new SmartJoystick(JoystickPorts.SECOND, state);
		} else {
			secondJoystick.setState(state);
		}
	}

	public void setThirdJoystickState(JoystickState state) {
		if (state == JoystickState.NONE) {
			thirdJoystick = null;
		} else if (thirdJoystick == null) {
			thirdJoystick = new SmartJoystick(JoystickPorts.THIRD, state);
		} else {
			thirdJoystick.setState(state);
		}
	}

	public void setFourthJoystickState(JoystickState state) {
		if (state == JoystickState.NONE) {
			fourthJoystick = null;
		} else if (fourthJoystick == null) {
			fourthJoystick = new SmartJoystick(JoystickPorts.FOURTH, state);
		} else {
			fourthJoystick.setState(state);
		}
	}

	public void setFifthJoystickState(JoystickState state) {
		if (state == JoystickState.NONE) {
			fifthJoystick = null;
		} else if (fifthJoystick == null) {
			fifthJoystick = new SmartJoystick(JoystickPorts.FIFTH, state);
		} else {
			fifthJoystick.setState(state);
		}
	}

	public void setSixthJoystickState(JoystickState state) {
		if (state == JoystickState.NONE) {
			sixthJoystick = null;
		} else if (sixthJoystick == null) {
			sixthJoystick = new SmartJoystick(JoystickPorts.SIXTH, state);
		} else {
			sixthJoystick.setState(state);
		}
	}

	public SmartJoystick getMainJoystick() {
		if (mainJoystick == null) {
			setMainJoystickState(JoystickState.EMPTY);
		}
		return mainJoystick;
	}

	public SmartJoystick getSecondJoystick() {
		if (secondJoystick == null) {
			setSecondJoystickState(JoystickState.EMPTY);
		}
		return secondJoystick;
	}

	public SmartJoystick getThirdJoystick() {
		if (thirdJoystick == null) {
			setThirdJoystickState(JoystickState.EMPTY);
		}
		return thirdJoystick;
	}

	public SmartJoystick getFourthJoystick() {
		if (fourthJoystick == null) {
			setFourthJoystickState(JoystickState.EMPTY);
		}
		return fourthJoystick;
	}

	public SmartJoystick getFifthJoystick() {
		if (fifthJoystick == null) {
			setFifthJoystickState(JoystickState.EMPTY);
		}
		return fifthJoystick;
	}

	public SmartJoystick getSixthJoystick() {
		if (sixthJoystick == null) {
			setSixthJoystickState(JoystickState.EMPTY);
		}
		return sixthJoystick;
	}

}
