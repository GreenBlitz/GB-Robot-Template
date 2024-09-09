package frc.utils.digitalinput.chooser;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.utils.digitalinput.DigitalInputInputsAutoLogged;
import frc.utils.digitalinput.IDigitalInput;

public class ChooserDigitalInput implements IDigitalInput {

	private final SendableChooser<Boolean> digitalInputValue;

	public ChooserDigitalInput(String chooserName) {
		this.digitalInputValue = new SendableChooser<>();
		digitalInputValue.setDefaultOption(
			String.valueOf(ChooserDigitalInputConstants.DEFAULT_STATE),
			ChooserDigitalInputConstants.DEFAULT_STATE
		);
		digitalInputValue
			.addOption(String.valueOf(!ChooserDigitalInputConstants.DEFAULT_STATE), !ChooserDigitalInputConstants.DEFAULT_STATE);
		SmartDashboard.putData(chooserName, digitalInputValue);
	}

	@Override
	public void updateInputs(DigitalInputInputsAutoLogged inputs) {
		inputs.nonDebouncedValue = digitalInputValue.getSelected();
		inputs.debouncedValue = inputs.nonDebouncedValue;
	}

}
