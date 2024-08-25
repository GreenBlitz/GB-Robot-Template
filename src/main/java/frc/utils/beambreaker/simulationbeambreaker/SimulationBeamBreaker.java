package frc.utils.beambreaker.simulationbeambreaker;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.utils.beambreaker.BeamBreakerInputsAutoLogged;
import frc.utils.beambreaker.IBeamBreaker;

import java.util.function.Consumer;

public class SimulationBeamBreaker implements IBeamBreaker {

	private Consumer<Boolean> isObstructedConsumer;
	private final SendableChooser<Boolean> isObstructedSendableChooser;
	private boolean isObstructed;

	public SimulationBeamBreaker() {
		this.isObstructedConsumer = this::setIsObstructed;
		this.isObstructedSendableChooser = new SendableChooser<Boolean>();
		this.isObstructed= false;
		isObstructedSendableChooser.onChange(isObstructedConsumer);
	}

	public void setIsObstructed(boolean isObstructed) {
		this.isObstructed=isObstructed;
	}

	@Override
	public void updateInputs(BeamBreakerInputsAutoLogged inputs) {
		inputs.isObstructed = isObstructed;
	}

}
