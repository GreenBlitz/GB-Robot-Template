package frc.robot.hardware.digitalinput.supplied;

import edu.wpi.first.math.filter.Debouncer;
import frc.robot.hardware.digitalinput.DigitalInputInputsAutoLogged;
import frc.robot.hardware.digitalinput.IDigitalInput;

import java.util.function.BooleanSupplier;

public class SuppliedDigitalInput implements IDigitalInput {

	private final BooleanSupplier booleanSupplier;
	private final Debouncer debouncer;
	private final boolean inverted;

	public SuppliedDigitalInput(BooleanSupplier booleanSupplier, Debouncer debouncer, boolean inverted) {
		this.booleanSupplier = booleanSupplier;
		this.debouncer = debouncer;
		this.inverted = inverted;
	}

	public SuppliedDigitalInput(BooleanSupplier booleanSupplier, Debouncer debouncer) {
		this(booleanSupplier, debouncer, false);
	}

	@Override
	public void updateInputs(DigitalInputInputsAutoLogged inputs) {
		inputs.nonDebouncedValue = inverted ^ booleanSupplier.getAsBoolean();
		inputs.debouncedValue = debouncer.calculate(inputs.nonDebouncedValue);
	}

}
