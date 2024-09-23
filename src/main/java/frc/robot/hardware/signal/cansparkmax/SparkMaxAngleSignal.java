package frc.robot.hardware.signal.cansparkmax;

import frc.robot.hardware.signal.AngleSignal;
import frc.utils.AngleUnit;

import java.util.function.Supplier;

public class SparkMaxAngleSignal extends AngleSignal implements ISparkMaxSignal {

	Supplier<Double> angleSignalSupplier;

	public SparkMaxAngleSignal(String name, Supplier<Double> angleSignalSupplier, AngleUnit angleUnit) {
		super(name, angleUnit);
		this.angleSignalSupplier = angleSignalSupplier;
	}

	@Override
	protected double getNewValue() {
		return angleSignalSupplier.get();
	}

}
