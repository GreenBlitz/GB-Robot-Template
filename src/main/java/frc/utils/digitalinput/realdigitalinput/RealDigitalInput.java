package frc.utils.digitalinput.realdigitalinput;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.utils.digitalinput.DigitalInputInputsAutoLogged;
import frc.utils.digitalinput.IDigitalInput;

public class RealDigitalInput implements IDigitalInput {

	private final DigitalInput digitalInput;

	private final Debouncer debouncer;

	public RealDigitalInput(int channel, double debounceTime, Debouncer.DebounceType debounceType) {
		this.digitalInput = new DigitalInput(channel);
		this.debouncer = new Debouncer(debounceTime, debounceType);
	}

	public void updateInputs(DigitalInputInputsAutoLogged inputs) {
		inputs.debouncedValue = debouncer.calculate(digitalInput.get());
		inputs.nonDebouncedValue = digitalInput.get();
	}

}
