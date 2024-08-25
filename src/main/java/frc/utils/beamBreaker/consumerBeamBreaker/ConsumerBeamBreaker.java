package frc.utils.beamBreaker.consumerBeamBreaker;

import frc.utils.beamBreaker.BeamBreakerInputsAutoLogged;
import frc.utils.beamBreaker.IBeamBreaker;

import java.util.function.Supplier;

public class ConsumerBeamBreaker implements IBeamBreaker {

	Supplier<Boolean> isObstructedSupplier;

	public ConsumerBeamBreaker(Supplier<Boolean> isObstructedSupplier) {
		this.isObstructedSupplier = isObstructedSupplier;
	}

	@Override
	public void updateInputs(BeamBreakerInputsAutoLogged inputs) {
		inputs.isObstructed = isObstructedSupplier.get();
	}

}
