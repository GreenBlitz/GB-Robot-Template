package frc.utils.digitalinput.supplieddigitalinput;

import edu.wpi.first.math.filter.Debouncer;
import frc.utils.digitalinput.DigitalInputInputsAutoLogged;
import frc.utils.digitalinput.IDigitalInput;

import java.util.function.Supplier;

public class SuppliedDigitalInput implements IDigitalInput {

	private final Debouncer debouncer;

	private final Supplier<Boolean> isObstructedConsumer;

	public SuppliedDigitalInput(
		Supplier<Boolean> isObstructedConsumer,
		double debounceTime,
		Debouncer.DebounceType debounceType
	) {
		this.isObstructedConsumer = isObstructedConsumer;
		this.debouncer = new Debouncer(debounceTime, debounceType);
	}

	@Override
	public void updateInputs(DigitalInputInputsAutoLogged inputs) {
		inputs.isObstructedWithDebouncer = debouncer.calculate(isObstructedConsumer.get());
		inputs.isObstructedWithoutDebouncer = isObstructedConsumer.get();
	}

}
