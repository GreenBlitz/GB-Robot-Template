package frc.utils.digitalinput.chooser;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.utils.digitalinput.DigitalInputInputsAutoLogged;
import frc.utils.digitalinput.IDigitalInput;

public class ChooserDigitalInput implements IDigitalInput {

	private SendableChooser<Boolean> sendableChooser;

	public ChooserDigitalInput(String chooserName) {
		this.sendableChooser = new SendableChooser<>();
		sendableChooser.setDefaultOption(
			String.valueOf(ChooserDigitalInputConstants.DEFAULT_STATE),
			ChooserDigitalInputConstants.DEFAULT_STATE
		);
		sendableChooser.addOption(String.valueOf(!ChooserDigitalInputConstants.DEFAULT_STATE), !ChooserDigitalInputConstants.DEFAULT_STATE);
		SmartDashboard.putData(chooserName, sendableChooser);
	}

	@Override
	public void updateInputs(DigitalInputInputsAutoLogged inputs) {
		inputs.nonDebouncedValue = sendableChooser.getSelected();
		inputs.debouncedValue = inputs.nonDebouncedValue;
	}

}
