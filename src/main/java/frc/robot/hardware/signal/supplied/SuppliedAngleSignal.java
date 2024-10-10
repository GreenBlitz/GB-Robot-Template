package frc.robot.hardware.signal.supplied;

import frc.robot.hardware.signal.AngleSignal;
import frc.robot.hardware.signal.TimedValue;
import frc.utils.AngleUnit;
import org.littletonrobotics.junction.Logger;

import java.util.function.Supplier;

public class SuppliedAngleSignal extends AngleSignal {

	private final Supplier<Double> angleSupplier;

	public SuppliedAngleSignal(String name, Supplier<Double> angleSupplier, AngleUnit angleUnit) {
		super(name, angleUnit);
		this.angleSupplier = angleSupplier;
	}

	@Override
	protected TimedValue<Double> getNewValue() {
		return new TimedValue<>(angleSupplier.get(), Logger.getRealTimestamp() / 1.0e6);
	}

}
