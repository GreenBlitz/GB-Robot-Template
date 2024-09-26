package frc.robot.hardware.signal.cansparkmax;

import edu.wpi.first.hal.HALUtil;
import frc.robot.hardware.signal.DoubleSignal;
import frc.robot.hardware.signal.TimedValue;
import frc.utils.Conversions;

import java.util.function.Supplier;

public class SparkMaxDoubleSignal extends DoubleSignal implements ISparkMaxSignal {

	private final Supplier<Double> doubleSupplier;

	public SparkMaxDoubleSignal(String name, Supplier<Double> doubleSupplier) {
		super(name);
		this.doubleSupplier = doubleSupplier;
	}

	@Override
	protected TimedValue<Double> getNewValue() {
		return new TimedValue<>(doubleSupplier.get(), Conversions.microSecondsToSeconds(HALUtil.getFPGATime()));
	}

}
