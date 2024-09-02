package frc.utils.digitalinput.channeled;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.utils.digitalinput.DigitalInputInputsAutoLogged;
import frc.utils.digitalinput.IDigitalInput;

public class ChanneledDigitalInput implements IDigitalInput {

	private final DigitalInput digitalInput;
	private final Debouncer debouncer;
	private final boolean normallyClosed;

	public ChanneledDigitalInput(int channel, double debounceTime, boolean normallyClosed) {
		this.digitalInput = new DigitalInput(channel);
		this.debouncer = new Debouncer(debounceTime);
		this.normallyClosed = normallyClosed;
	}

	@Override
	public void updateInputs(DigitalInputInputsAutoLogged inputs) {
		if (normallyClosed) {
			inputs.debouncedValue = !debouncer.calculate(digitalInput.get());
			inputs.nonDebouncedValue = !digitalInput.get();
		} else {
			inputs.debouncedValue = debouncer.calculate(digitalInput.get());
			inputs.nonDebouncedValue = digitalInput.get();
		}
	}

}
