package frc.robot.hardware.signal.cansparkmax;

import frc.robot.hardware.signal.AngleSignal;
import frc.robot.hardware.signal.TimedValue;
import frc.utils.AngleUnit;

import java.util.function.Supplier;

public class SparkMaxAngleSignal extends AngleSignal implements ISparkMaxSignal {

	private final Supplier<Double> valueSupplier;

	public SparkMaxAngleSignal(String name, Supplier<Double> angleSignalSupplier, AngleUnit angleUnit) {
		super(name, angleUnit);
		this.valueSupplier = angleSignalSupplier;
	}

	@Override
	protected TimedValue<Double> getNewValue() {
		return new TimedValue<>(valueSupplier.get(), System.currentTimeMillis());
	}

}
