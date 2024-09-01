package frc.utils.digitalinput.channeled;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.utils.digitalinput.DigitalInputInputsAutoLogged;
import frc.utils.digitalinput.IDigitalInput;

public class ChanneledDigitalInput implements IDigitalInput {

	private final DigitalInput digitalInput;

	private final Debouncer debouncer;

	public ChanneledDigitalInput(int channel, double debounceTime, Debouncer.DebounceType debounceType) {
		this.digitalInput = new DigitalInput(channel);
		this.debouncer = new Debouncer(debounceTime);
	}

	@Override
	public void updateInputs(DigitalInputInputsAutoLogged inputs) {
		inputs.debouncedValue = debouncer.calculate(digitalInput.get());
		inputs.nonDebouncedValue = digitalInput.get();
	}

}
