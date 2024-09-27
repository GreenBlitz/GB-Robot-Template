package frc.robot.hardware.signal.cansparkmax;

import edu.wpi.first.hal.HALUtil;
import frc.robot.hardware.signal.AngleSignal;
import frc.robot.hardware.signal.TimedValue;
import frc.utils.AngleUnit;
import frc.utils.Conversions;

import java.util.function.Supplier;

public class SparkMaxAngleSignal extends AngleSignal implements ISparkMaxSignal {

	private final Supplier<Double> angleSupplier;

	public SparkMaxAngleSignal(String name, Supplier<Double> angleSupplier, AngleUnit angleUnit) {
		super(name, angleUnit);
		this.angleSupplier = angleSupplier;
	}

	@Override
	protected TimedValue<Double> getNewValue() {
		return new TimedValue<>(angleSupplier.get(), Conversions.microSecondsToSeconds(HALUtil.getFPGATime()));
	}

}
