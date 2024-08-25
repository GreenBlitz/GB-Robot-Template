package frc.utils.beamBreaker.digitalBeamBreaker;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.utils.beamBreaker.BeamBreakerInputsAutoLogged;
import frc.utils.beamBreaker.IBeamBreaker;

public class DigitalBeamBreaker implements IBeamBreaker {

	private final DigitalInput beambreaker;

	private final Debouncer debouncer;

	public DigitalBeamBreaker(int DigitalInputChannel, double debounceTime, Debouncer.DebounceType DebounceType) {
		this.beambreaker = new DigitalInput(DigitalInputChannel);
		this.debouncer = new Debouncer(debounceTime, DebounceType);
	}

	@Override
	public void updateInputs(BeamBreakerInputsAutoLogged inputs) {
		inputs.isObstructed = debouncer.calculate(beambreaker.get());
	}

}
