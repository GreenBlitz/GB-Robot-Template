package frc.utils.digitalinput.supplied;

import edu.wpi.first.math.filter.Debouncer;
import frc.utils.digitalinput.DigitalInputInputsAutoLogged;
import frc.utils.digitalinput.IDigitalInput;

import java.util.function.BooleanSupplier;

public class SuppliedDigitalInput implements IDigitalInput {

	private final Debouncer debouncer;
	private final boolean inverted;
	private final BooleanSupplier booleanSupplier;

	public SuppliedDigitalInput(
		BooleanSupplier booleanSupplier,
		double debounceTime,
		Debouncer.DebounceType debounceType,
		boolean inverted
	) {
		this.booleanSupplier = booleanSupplier;
		this.debouncer = new Debouncer(debounceTime, debounceType);
		this.inverted = inverted;
	}

	public SuppliedDigitalInput(BooleanSupplier booleanSupplier, double debounceTime, Debouncer.DebounceType debounceType) {
		this(booleanSupplier, debounceTime, debounceType, false);
	}

	@Override
	public void updateInputs(DigitalInputInputsAutoLogged inputs) {
		inputs.nonDebouncedValue = inverted ? !booleanSupplier.getAsBoolean() : booleanSupplier.getAsBoolean();
		inputs.debouncedValue = debouncer.calculate(inputs.nonDebouncedValue);
	}

}
