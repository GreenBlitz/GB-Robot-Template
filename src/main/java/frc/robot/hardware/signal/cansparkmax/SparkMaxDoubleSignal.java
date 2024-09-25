package frc.robot.hardware.signal.cansparkmax;

import frc.robot.hardware.signal.DoubleSignal;
import frc.robot.hardware.signal.TimedValue;

import java.util.function.Supplier;

public class SparkMaxDoubleSignal extends DoubleSignal implements ISparkMaxSignal {

	private final Supplier<Double> valueSupplier;

	public SparkMaxDoubleSignal(String name, Supplier<Double> doubleSignalSupplier) {
		super(name);
		this.valueSupplier = doubleSignalSupplier;
	}

	@Override
	protected TimedValue<Double> getNewValue() {
		return new TimedValue<>(valueSupplier.get(), System.currentTimeMillis());
	}

}
