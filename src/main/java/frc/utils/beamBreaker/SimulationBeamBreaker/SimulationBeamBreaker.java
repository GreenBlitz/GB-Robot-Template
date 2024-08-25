package frc.utils.beamBreaker.SimulationBeamBreaker;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.utils.beamBreaker.BeamBreakerInputsAutoLogged;
import frc.utils.beamBreaker.IBeamBreaker;

import java.util.function.Consumer;

public class SimulationBeamBreaker implements IBeamBreaker {


	Consumer<Boolean> isObstructedConsumer = obstructed -> setIsObstructed(obstructed);
	public SendableChooser<Boolean> isObstructedSendableChooser;
	public boolean isObstructed = false;

	public SimulationBeamBreaker() {
		isObstructedSendableChooser = new SendableChooser<Boolean>();
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
