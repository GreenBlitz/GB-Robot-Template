package frc.utils.beambreaker.digitalbeambreaker;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.utils.beambreaker.IDigitalInput;
import frc.utils.beambreaker.digitalInputAutoLogged;

public class DigitalBeamBreaker implements IDigitalInput {

	private final DigitalInput beambreaker;
	private final Debouncer debouncer;

	public DigitalBeamBreaker(int channel, double debounceTime, Debouncer.DebounceType debounceType) {
		this.beambreaker = new DigitalInput(channel);
		this.debouncer = new Debouncer(debounceTime, debounceType);
	}

	@Override
	public void updateInputs(digitalInputAutoLogged inputs) {
		inputs.isObstructed = debouncer.calculate(beambreaker.get());
	}

}
