package frc.robot.hardware.signal.supplied;

import edu.wpi.first.hal.HALUtil;
import frc.robot.hardware.signal.DoubleSignal;
import frc.robot.hardware.signal.TimedValue;
import frc.utils.Conversions;
import org.littletonrobotics.junction.Logger;

import java.util.function.Supplier;

public class SuppliedDoubleSignal extends DoubleSignal {

	private final Supplier<Double> doubleSupplier;

	public SuppliedDoubleSignal(String name, Supplier<Double> doubleSupplier) {
		super(name);
		this.doubleSupplier = doubleSupplier;
	}

	@Override
	protected TimedValue<Double> getNewValue() {
		return new TimedValue<>(doubleSupplier.get(), Logger.getRealTimestamp() / 1.0e6);
	}

}
