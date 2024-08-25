package frc.utils.beambreaker.simulationbeambreaker;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.utils.beambreaker.BeamBreakerInputsAutoLogged;
import frc.utils.beambreaker.IBeamBreaker;

import java.util.function.Consumer;

public class SimulationBeamBreaker implements IBeamBreaker {

	private final Consumer<Boolean> isObstructedConsumer = obstructed -> setIsObstructed(obstructed);
	private final SendableChooser<Boolean> isObstructedSendableChooser;
	private boolean isObstructed = false;

	public SimulationBeamBreaker() {
		this.isObstructedSendableChooser = new SendableChooser<Boolean>();
		isObstructedSendableChooser.onChange(isObstructedConsumer);
	}

	public void setIsObstructed(boolean isObstructed) {
		isObstructedConsumer.accept(isObstructed);
	}

	@Override
	public void updateInputs(BeamBreakerInputsAutoLogged inputs) {
		inputs.isObstructed = isObstructed;
	}

}
