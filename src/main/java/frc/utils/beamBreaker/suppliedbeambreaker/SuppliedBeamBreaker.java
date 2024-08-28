package frc.utils.beambreaker.suppliedbeambreaker;

import edu.wpi.first.math.filter.Debouncer;
import frc.utils.beambreaker.IDigitalInput;
import frc.utils.beambreaker.digitalInputAutoLogged;
import java.util.function.Supplier;

public class SuppliedBeamBreaker implements IDigitalInput {

	private final Debouncer debouncer;
	private final Supplier<Boolean> isObstructedConsumer;

	public SuppliedBeamBreaker(Supplier<Boolean> isObstructedConsumer, double debounceTime, Debouncer.DebounceType debounceType) {
		this.isObstructedConsumer = isObstructedConsumer;
		this.debouncer = new Debouncer(debounceTime, debounceType);
	}

@Override
	public void updateInputs(digitalInputAutoLogged inputs) {
		inputs.isObstructed = debouncer.calculate(isObstructedConsumer.get());
	}

}
