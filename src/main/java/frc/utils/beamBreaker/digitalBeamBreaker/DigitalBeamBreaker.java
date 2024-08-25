package frc.utils.beamBreaker.digitalBeamBreaker;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.utils.beamBreaker.BeamBreakerInputsAutoLogged;
import frc.utils.beamBreaker.IBeamBreaker;

public class DigitalBeamBreaker implements IBeamBreaker {

	private static DigitalInput input;

	private Debouncer debouncer;

	private boolean isObstructed = false;

	public DigitalBeamBreaker() {
		input = new DigitalInput(0);
		debouncer = new Debouncer(0.1, Debouncer.DebounceType.kBoth);
	}

	@Override
	public void updateInputs(BeamBreakerInputsAutoLogged inputs) {
		inputs.isObstructed = debouncer.calculate(input.get());
	}

}
