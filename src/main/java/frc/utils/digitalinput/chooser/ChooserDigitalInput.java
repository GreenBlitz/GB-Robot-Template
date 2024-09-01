package frc.utils.digitalinput.chooser;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.utils.digitalinput.DigitalInputInputsAutoLogged;
import frc.utils.digitalinput.IDigitalInput;

import java.util.function.Consumer;

public class ChooserDigitalInput implements IDigitalInput {

	private boolean isTrue;

	public ChooserDigitalInput() {
		SendableChooser<Boolean> isTrueSendableChooser = new SendableChooser<>();
		isTrueSendableChooser.addOption("true", true);
		isTrueSendableChooser.addOption("false", false);

		this.isTrue = ChooserDigitalInputConstants.DEFAULT_STATE;
		SmartDashboard.putData(isTrueSendableChooser);
	}

	public void setIsTrue(boolean isTrue) {
		this.isTrue = isTrue;
	}

	@Override
	public void updateInputs(DigitalInputInputsAutoLogged inputs) {
		inputs.debouncedValue = isTrue;
		inputs.nonDebouncedValue = isTrue;
	}

}
