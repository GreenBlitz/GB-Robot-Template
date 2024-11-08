package frc.robot.hard.signal.supplied;

import frc.robot.hard.signal.DoubleSignal;
import frc.robot.hard.signal.TimedValue;
import frc.utils.time.TimeUtils;

import java.util.function.Supplier;

public class SuppliedDoubleSignal extends DoubleSignal {

	private final Supplier<Double> doubleSupplier;

	public SuppliedDoubleSignal(String name, Supplier<Double> doubleSupplier) {
		super(name);
		this.doubleSupplier = doubleSupplier;
	}

	@Override
	protected TimedValue<Double> getNewValue() {
		return new TimedValue<>(doubleSupplier.get(), TimeUtils.getCurrentTimeSeconds());
	}

}
