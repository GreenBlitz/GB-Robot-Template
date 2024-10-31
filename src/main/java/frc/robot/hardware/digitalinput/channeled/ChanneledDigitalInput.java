package frc.robot.hardware.digitalinput.channeled;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.hardware.digitalinput.DigitalInputInputsAutoLogged;
import frc.robot.hardware.digitalinput.IDigitalInput;

public class ChanneledDigitalInput implements IDigitalInput {

	private final DigitalInput digitalInput;
	private final Debouncer debouncer;
	private final boolean inverted;

	public ChanneledDigitalInput(int channel, Debouncer debouncer, boolean inverted) {
		this.digitalInput = new DigitalInput(channel);
		this.debouncer = debouncer;
		this.inverted = inverted;
	}

	public ChanneledDigitalInput(int channel, Debouncer debouncer) {
		this(channel, debouncer, false);
	}

	@Override
	public void updateInputs(DigitalInputInputsAutoLogged inputs) {
		inputs.nonDebouncedValue = inverted ? !digitalInput.get() : digitalInput.get();
		inputs.debouncedValue = debouncer.calculate(inputs.nonDebouncedValue);
	}

}
