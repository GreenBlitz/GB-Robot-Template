package frc.utils.beamBreaker.suppliedbeambreaker;

import edu.wpi.first.math.filter.Debouncer;
import frc.utils.beamBreaker.BeamBreakerInputsAutoLogged;
import frc.utils.beamBreaker.IBeamBreaker;

import java.util.function.Supplier;

public class SuppliedBeamBreaker implements IBeamBreaker {

	private final Debouncer debouncer;
	private final Supplier<Boolean> isObstructedConsumer;

	public SuppliedBeamBreaker(Supplier<Boolean> isObstructedConsumer, double debounceTime, Debouncer.DebounceType debounceType) {
		this.isObstructedConsumer = isObstructedConsumer;
		this.debouncer = new Debouncer(debounceTime, debounceType);
	}

	@Override
	public void updateInputs(BeamBreakerInputsAutoLogged inputs) {
		inputs.isObstructed = debouncer.calculate(isObstructedConsumer.get());
	}

}
