package frc.robot.hardware.digitalinput.channeled;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.hardware.digitalinput.DigitalInputInputsAutoLogged;
import frc.robot.hardware.digitalinput.IDigitalInput;

public class ChanneledDigitalInput implements IDigitalInput {

	private final DigitalInput digitalInput;
	private final Debouncer debouncer;
	private final boolean inverted;

	public ChanneledDigitalInput(DigitalInput digitalInput, Debouncer debouncer, boolean inverted) {
		this.digitalInput = digitalInput;
		this.debouncer = debouncer;
		this.inverted = inverted;
	}

	public ChanneledDigitalInput(DigitalInput digitalInput, Debouncer debouncer) {
		this(digitalInput, debouncer, false);
	}

	@Override
	public void updateInputs(DigitalInputInputsAutoLogged inputs) {
		inputs.nonDebouncedValue = inverted ^ digitalInput.get();
		inputs.debouncedValue = debouncer.calculate(inputs.nonDebouncedValue);
	}

}
