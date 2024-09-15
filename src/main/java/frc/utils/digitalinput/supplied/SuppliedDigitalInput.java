package frc.utils.digitalinput.supplied;

import edu.wpi.first.math.filter.Debouncer;
import frc.utils.digitalinput.DigitalInputInputsAutoLogged;
import frc.utils.digitalinput.IDigitalInput;

import java.util.function.BooleanSupplier;

public class SuppliedDigitalInput implements IDigitalInput {

	private final BooleanSupplier booleanSupplier;
	private final Debouncer debouncer;
	private final boolean inverted;

	public SuppliedDigitalInput(
		BooleanSupplier booleanSupplier,
		Debouncer.DebounceType debounceType,
		double debounceTime,
		boolean inverted
	) {
		this.booleanSupplier = booleanSupplier;
		this.debouncer = new Debouncer(debounceTime, debounceType);
		this.inverted = inverted;
	}

	public SuppliedDigitalInput(BooleanSupplier booleanSupplier, Debouncer.DebounceType debounceType, double debounceTime) {
		this(booleanSupplier, debounceType, debounceTime, false);
	}

	@Override
	public void updateInputs(DigitalInputInputsAutoLogged inputs) {
		inputs.nonDebouncedValue = inverted ? !booleanSupplier.getAsBoolean() : booleanSupplier.getAsBoolean();
		inputs.debouncedValue = debouncer.calculate(inputs.nonDebouncedValue);
	}

}
