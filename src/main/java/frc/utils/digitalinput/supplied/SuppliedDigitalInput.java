package frc.utils.digitalinput.supplied;

import edu.wpi.first.math.filter.Debouncer;
import frc.utils.digitalinput.DigitalInputInputsAutoLogged;
import frc.utils.digitalinput.IDigitalInput;

import java.util.function.Supplier;

public class SuppliedDigitalInput implements IDigitalInput {

	private final Debouncer debouncer;

	private final Supplier<Boolean> isTrueSupplier;

	public SuppliedDigitalInput(Supplier<Boolean> isTrueConsumer, double debounceTime, Debouncer.DebounceType debounceType) {
		this.isTrueSupplier = isTrueConsumer;
		this.debouncer = new Debouncer(debounceTime, debounceType);
	}

	@Override
	public void updateInputs(DigitalInputInputsAutoLogged inputs) {
		inputs.debouncedValue = debouncer.calculate(isTrueSupplier.get());
		inputs.nonDebouncedValue = isTrueSupplier.get();
	}

}
