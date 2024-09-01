package frc.utils.digitalinput.simulationdigitalinput;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.utils.digitalinput.DigitalInputInputsAutoLogged;
import frc.utils.digitalinput.IDigitalInput;

import java.util.function.Consumer;

public class SimulationDigitalInput implements IDigitalInput {

	private final Consumer<Boolean> isTrueConsumer = this::setIsTrue;

	private boolean isTrue;

	public SimulationDigitalInput() {
		SendableChooser<Boolean> isTrueSendableChooser = new SendableChooser<>();
		this.isTrue = SimulationDigitalInputConstants.DEFAULT_STATE;
		isTrueSendableChooser.onChange(isTrueConsumer);
	}

	public void setIsTrue(boolean isTrue) {
		this.isTrue = isTrue;
	}

}
