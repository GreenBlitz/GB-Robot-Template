package frc.utils.beamBreaker.consumerBeamBreaker;

import frc.utils.beamBreaker.BeamBreakerInputsAutoLogged;
import frc.utils.beamBreaker.IBeamBreaker;

import java.util.function.Supplier;

public class ConsumerBeamBreaker implements IBeamBreaker {

	Supplier<Boolean> isObstructedConsumer;

	public ConsumerBeamBreaker(Supplier<Boolean> isObstructedConsumer) {
		this.isObstructedConsumer = isObstructedConsumer;
	}

	@Override
	public void updateInputs(BeamBreakerInputsAutoLogged inputs) {
		inputs.isObstructed = isObstructedConsumer.get();
	}


}
