package frc.utils.digitalinput.simulationdigitalinput;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.utils.digitalinput.DigitalInputInputsAutoLogged;
import frc.utils.digitalinput.IDigitalInput;

import java.util.function.Consumer;

public class SimulationDigitalInput implements IDigitalInput {

	Consumer<Boolean> isTrueConsumer = this::setIsTrue;

	private boolean isTrue;

	public SimulationDigitalInput() {
		final SendableChooser<Boolean> isTrueSendableChooser = new SendableChooser<>();
		this.isTrue = SimulationDigitalInputConstants.DEFAULT_STATE;
		isTrueSendableChooser.onChange(isTrueConsumer);
	}

	public void setIsTrue(boolean isTrue) {
		this.isTrue = isTrue;
	}

	@Override
	public void updateInputs(DigitalInputInputsAutoLogged inputs) {
		isTrueConsumer.accept(SmartDashboard.getBoolean("is obstructed", SimulationDigitalInputConstants.DEFAULT_STATE));
		inputs.isTrueWithDebouncer = isTrue;
		inputs.isTrueWithoutDebouncer = isTrue;
	}

}
