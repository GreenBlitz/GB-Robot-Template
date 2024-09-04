package frc.utils.digitalinput.supplied;

import edu.wpi.first.math.filter.Debouncer;
import frc.utils.digitalinput.DigitalInputInputsAutoLogged;
import frc.utils.digitalinput.IDigitalInput;

import java.util.function.Supplier;

public class SuppliedDigitalInput implements IDigitalInput {

	private final Debouncer debouncer;
	private boolean inverted = false;

	private final Supplier<Boolean> isTrueSupplier;

	public SuppliedDigitalInput(
		Supplier<Boolean> isTrueConsumer,
		double debounceTime,
		Debouncer.DebounceType debounceType,
		boolean inverted
	) {
		this.isTrueSupplier = isTrueConsumer;
		this.debouncer = new Debouncer(debounceTime, debounceType);
		this.inverted = inverted;
	}

	public SuppliedDigitalInput(Supplier<Boolean> isTrueConsumer, double debounceTime, Debouncer.DebounceType debounceType) {
		this.isTrueSupplier = isTrueConsumer;
		this.debouncer = new Debouncer(debounceTime, debounceType);
	}

	@Override
	public void setInverted(boolean inverted) {
		this.inverted = inverted;
	}

	@Override
	public void updateInputs(DigitalInputInputsAutoLogged inputs) {
		inputs.nonDebouncedValue = (inverted) ? !isTrueSupplier.get() : isTrueSupplier.get();
		inputs.debouncedValue = debouncer.calculate(inputs.nonDebouncedValue);
	}


}
