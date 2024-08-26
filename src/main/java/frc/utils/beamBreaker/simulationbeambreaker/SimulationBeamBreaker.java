package frc.utils.beamBreaker.simulationbeambreaker;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.utils.beamBreaker.BeamBreakerInputsAutoLogged;
import frc.utils.beamBreaker.IBeamBreaker;

import java.util.function.Consumer;

public class SimulationBeamBreaker implements IBeamBreaker {
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
	public void updateInputs(BeamBreakerInputsAutoLogged inputs) {
		inputs.isObstructed = isObstructed;
	}

}
