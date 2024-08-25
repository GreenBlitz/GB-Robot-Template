package frc.utils.beamBreaker.digitalBeamBreaker;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.utils.beamBreaker.BeamBreakerInputsAutoLogged;
import frc.utils.beamBreaker.IBeamBreaker;

public class DigitalBeamBreaker implements IBeamBreaker {

	private static DigitalInput input;

	private Debouncer debouncer;

	public DigitalBeamBreaker() {
		input = new DigitalInput(0);
		debouncer = new Debouncer(0.1, Debouncer.DebounceType.kBoth);
	}

	public void setIsObstructed(boolean isObstructed) {
	}

	@Override
	public void updateInputs(BeamBreakerInputsAutoLogged inputs) {
		inputs.isObstructed = debouncer.calculate(input.get());
	}

}
