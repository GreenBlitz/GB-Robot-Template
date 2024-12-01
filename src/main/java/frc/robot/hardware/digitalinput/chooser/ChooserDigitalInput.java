package frc.robot.hardware.digitalinput.chooser;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.hardware.digitalinput.DigitalInputInputsAutoLogged;
import frc.robot.hardware.digitalinput.IDigitalInput;

public class ChooserDigitalInput implements IDigitalInput {

	private final SendableChooser<Boolean> digitalInputValueChooser;

	public ChooserDigitalInput(String chooserName) {
		this.digitalInputValueChooser = new SendableChooser<>();

		digitalInputValueChooser.setDefaultOption("false", false);
		digitalInputValueChooser.addOption("true", true);
		SmartDashboard.putData(chooserName, digitalInputValueChooser);
	}

	@Override
	public void updateInputs(DigitalInputInputsAutoLogged inputs) {
		inputs.nonDebouncedValue = digitalInputValueChooser.getSelected();
		inputs.debouncedValue = inputs.nonDebouncedValue;
	}

}
