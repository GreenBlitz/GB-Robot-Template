package frc.robot.hardware.digitalinput.chooser;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.hardware.digitalinput.DigitalInputInputsAutoLogged;
import frc.robot.hardware.digitalinput.IDigitalInput;

public class ChooserDigitalInput implements IDigitalInput {

	private final SendableChooser<Boolean> digitalInputValue;

	public ChooserDigitalInput(String chooserName) {
		this.digitalInputValue = new SendableChooser<>();

		digitalInputValue.setDefaultOption("false", false);
		digitalInputValue.addOption("true", true);
		SmartDashboard.putData(chooserName, digitalInputValue);
	}

	@Override
	public void updateInputs(DigitalInputInputsAutoLogged inputs) {
		inputs.nonDebouncedValue = digitalInputValue.getSelected();
		inputs.debouncedValue = inputs.nonDebouncedValue;
	}

}
