package frc.utils.digitalinput.simulationdigitalinput;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.utils.digitalinput.DigitalInputInputsAutoLogged;
import frc.utils.digitalinput.IDigitalInput;

import java.util.function.Consumer;

public class SimulationDigitalInput implements IDigitalInput {

	Consumer<Boolean> isObstructedConsumer = this::setIsObstructed;

	private boolean isObstructed;

	public SimulationDigitalInput() {
		final SendableChooser<Boolean> isObstructedSendableChooser = new SendableChooser<>();
		this.isObstructed = SimulationDigitalInputConstants.DEFAULT_STATE;
		isObstructedSendableChooser.onChange(isObstructedConsumer);
	}

	public void setIsObstructed(boolean isObstructed) {
		this.isObstructed = isObstructed;
	}

	@Override
	public void updateInputs(DigitalInputInputsAutoLogged inputs) {
		isObstructedConsumer.accept(SmartDashboard.getBoolean("is obstructed", SimulationDigitalInputConstants.DEFAULT_STATE));
		inputs.isObstructedWithoutDebouncer = isObstructed;
		inputs.isObstructedWithDebouncer = isObstructed;
	}

}
