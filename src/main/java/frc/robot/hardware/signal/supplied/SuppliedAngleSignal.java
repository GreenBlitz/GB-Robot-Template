package frc.robot.hardware.signal.supplied;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.hardware.signal.AngleSignal;
import frc.utils.math.AngleUnit;
import frc.utils.TimedValue;
import frc.utils.time.TimeUtil;
import java.util.function.Supplier;

public class SuppliedAngleSignal extends AngleSignal {

	private final Supplier<Double> angleSupplier;

	public SuppliedAngleSignal(String name, Supplier<Double> angleSupplier, AngleUnit angleUnit) {
		super(name, angleUnit);
		this.angleSupplier = angleSupplier;
	}

	@Override
	protected void updateValue(TimedValue<Rotation2d> timedValue) {
		timedValue.setValue(angleUnit.toRotation2d(angleSupplier.get()));
		timedValue.setTimestamp(TimeUtil.getCurrentTimeSeconds());
	}

}
