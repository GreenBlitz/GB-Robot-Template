package frc.robot.hard.signal.supplied;

import frc.robot.hard.signal.AngleSignal;
import frc.robot.hard.signal.TimedValue;
import frc.utils.AngleUnit;
import frc.utils.time.TimeUtils;

import java.util.function.Supplier;

public class SuppliedAngleSignal extends AngleSignal {

	private final Supplier<Double> angleSupplier;

	public SuppliedAngleSignal(String name, Supplier<Double> angleSupplier, AngleUnit angleUnit) {
		super(name, angleUnit);
		this.angleSupplier = angleSupplier;
	}

	@Override
	protected TimedValue<Double> getNewValue() {
		return new TimedValue<>(angleSupplier.get(), TimeUtils.getCurrentTimeSeconds());
	}

}
