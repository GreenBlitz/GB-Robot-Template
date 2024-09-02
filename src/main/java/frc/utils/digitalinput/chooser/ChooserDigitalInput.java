package frc.utils.digitalinput.chooser;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.utils.digitalinput.DigitalInputInputsAutoLogged;
import frc.utils.digitalinput.IDigitalInput;


public class ChooserDigitalInput implements IDigitalInput {

	private final SendableChooser<Boolean> isTrueSendableChooser;

	public ChooserDigitalInput() {
		isTrueSendableChooser = new SendableChooser<>();
		isTrueSendableChooser
			.setDefaultOption(ChooserDigitalInputConstants.DEFAULT_STATE_STRING, ChooserDigitalInputConstants.DEFAULT_STATE);
		isTrueSendableChooser.addOption("true", true);
		isTrueSendableChooser.addOption("false", false);

		SmartDashboard.putData(isTrueSendableChooser);
	}


	@Override
	public void updateInputs(DigitalInputInputsAutoLogged inputs) {
		inputs.debouncedValue = isTrueSendableChooser.getSelected();
		inputs.nonDebouncedValue = isTrueSendableChooser.getSelected();
	}

}
