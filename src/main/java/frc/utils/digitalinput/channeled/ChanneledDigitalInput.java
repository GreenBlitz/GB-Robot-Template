package frc.utils.digitalinput.channeled;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.utils.digitalinput.DigitalInputInputsAutoLogged;
import frc.utils.digitalinput.IDigitalInput;

public class ChanneledDigitalInput implements IDigitalInput {

	private final DigitalInput digitalInput;
	private final Debouncer debouncer;
	private final boolean inverted;

	public ChanneledDigitalInput(int channel, double debounceTime, boolean inverted) {
		this.digitalInput = new DigitalInput(channel);
		this.debouncer = new Debouncer(debounceTime);
		this.inverted = inverted;
	}

	public ChanneledDigitalInput(int channel, double debounceTime) {
		this(channel, debounceTime, false);
	}

	@Override
	public void updateInputs(DigitalInputInputsAutoLogged inputs) {
		inputs.nonDebouncedValue = inverted ? !digitalInput.get() : digitalInput.get();
		inputs.debouncedValue = debouncer.calculate(inputs.nonDebouncedValue);
	}

}
