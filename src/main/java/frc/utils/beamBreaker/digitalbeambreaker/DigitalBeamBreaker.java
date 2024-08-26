package frc.utils.beamBreaker.digitalbeambreaker;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.utils.beamBreaker.BeamBreakerInputsAutoLogged;
import frc.utils.beamBreaker.IBeamBreaker;

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
