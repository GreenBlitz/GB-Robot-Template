package frc.utils.beambreaker.simulationbeambreaker;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.utils.beambreaker.IDigitalInput;
import frc.utils.beambreaker.digitalInputAutoLogged;


import java.util.function.Consumer;

public class SimulationBeamBreaker implements IDigitalInput {

	private boolean isObstructed;

	public SimulationBeamBreaker() {
		Consumer<Boolean> isObstructedConsumer = this::setIsObstructed;
		this.isObstructed = false;
		SendableChooser<Boolean> isObstructedSendableChooser = new SendableChooser<Boolean>();
		isObstructedSendableChooser.onChange(isObstructedConsumer);
	}

	public void setIsObstructed(boolean isObstructed) {
		this.isObstructed = isObstructed;
	}

	@Override
	public void updateInputs(digitalInputAutoLogged inputs) {
		inputs.isObstructed = isObstructed;
	}

}
