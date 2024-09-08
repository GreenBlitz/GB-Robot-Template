package frc.robot.hardware.signal;

import edu.wpi.first.math.geometry.Rotation2d;

import java.util.Arrays;
import java.util.function.DoubleFunction;

public abstract class AngleSignal extends InputSignal<Rotation2d> {

	public enum AngleUnit {

		ROTATIONS(Rotation2d::fromRotations),
		RADIANS(Rotation2d::fromRadians),
		DEGREES(Rotation2d::fromDegrees);

		private final DoubleFunction<Rotation2d> toAngle;

		AngleUnit(DoubleFunction<Rotation2d> toAngle) {
			this.toAngle = toAngle;
		}

		public Rotation2d toAngle(double value) {
			return toAngle.apply(value);
		}

	}

	protected final AngleUnit angleUnit;

	protected AngleSignal(AngleUnit angleUnit) {
		super(new Rotation2d());
		this.angleUnit = angleUnit;
	}

	protected void setNewValues(double... newValues) {
		setNewValues(Arrays.stream(newValues).mapToObj(angleUnit::toAngle).toArray(Rotation2d[]::new));
	}

}
