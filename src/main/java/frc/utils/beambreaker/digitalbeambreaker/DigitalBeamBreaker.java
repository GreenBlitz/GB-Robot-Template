package frc.utils.beambreaker.digitalbeambreaker;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.utils.beambreaker.BeamBreakerInputsAutoLogged;
import frc.utils.beambreaker.IBeamBreaker;

public class DigitalBeamBreaker implements IBeamBreaker {

	private final DigitalInput beambreaker;

	private final Debouncer debouncer;

	public DigitalBeamBreaker(int channel, double debounceTime, Debouncer.DebounceType debounceType) {
		this.beambreaker = new DigitalInput(channel);
		this.debouncer = new Debouncer(debounceTime, debounceType);
	}

	@Override
	public void updateInputs(BeamBreakerInputsAutoLogged inputs) {
		inputs.isObstructed = debouncer.calculate(beambreaker.get());
	}

}
