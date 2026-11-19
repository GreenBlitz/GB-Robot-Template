package frc.utils.angle2d;

import static edu.wpi.first.units.Units.Radians;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.interpolation.Interpolatable;
import edu.wpi.first.math.MathSharedStore;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.util.protobuf.ProtobufSerializable;
import edu.wpi.first.util.struct.StructSerializable;

import java.util.Objects;

public class Angle2d implements Interpolatable<Angle2d>, ProtobufSerializable, StructSerializable {

	public static final Angle2d ZERO = new Angle2d();

	public static final Angle2d CLOCKWISE_HALF_PI = new Angle2d(-Math.PI / 2);

	public static final Angle2d CLOCKWISE_90DEG = CLOCKWISE_HALF_PI;

	public static final Angle2d COUNTER_CLOCKWISE_HALf_PI = new Angle2d(Math.PI / 2);

	public static final Angle2d COUNTER_CLOCKWISE_90DEG = COUNTER_CLOCKWISE_HALf_PI;

	public static final Angle2d COUNTER_CLOCKWISE_PI = new Angle2d(Math.PI);

	public static final Angle2d COUNTER_CLOCKWISE_180DEG = COUNTER_CLOCKWISE_PI;

	public static final Angle2d CLOCKWISE_PI = new Angle2d(Math.PI);

	public static final Angle2d CLOCKWISE_180DEG = CLOCKWISE_PI;

	private final double valueRads;
	private final double cos;
	private final double sin;

	public Angle2d() {
		valueRads = 0.0;
		cos = 1.0;
		sin = 0.0;
	}

	public Angle2d(double valueRads) {
		this.valueRads = valueRads;
		cos = Math.cos(valueRads);
		sin = Math.sin(valueRads);
	}

	public Angle2d(double x, double y) {
		double magnitude = Math.hypot(x, y);
		if (magnitude > 1e-6) {
			cos = x / magnitude;
			sin = y / magnitude;
		} else {
			cos = 1.0;
			sin = 0.0;
			MathSharedStore.reportError("x and y components of Angle2d are zero\n", Thread.currentThread().getStackTrace());
		}
		valueRads = Math.atan2(sin, cos);
	}

	public Angle2d(Angle angle) {
		this(angle.in(Radians));
	}

	public static Angle2d fromRadians(double radians) {
		return new Angle2d(radians);
	}

	public static Angle2d fromDegrees(double degrees) {
		return new Angle2d(Math.toRadians(degrees));
	}

	public static Angle2d fromRotations(double rotations) {
		return new Angle2d(Units.rotationsToRadians(rotations));
	}

	public static Angle2d fromRotation2d(Rotation2d rotation2d) {
		return Angle2d.fromRadians(rotation2d.getRadians());
	}

	public Angle2d plus(Angle2d other) {
		return rotateBy(other);
	}

	public Angle2d minus(Angle2d other) {
		return rotateBy(other.unaryMinus());
	}

	public Angle2d unaryMinus() {
		return new Angle2d(-valueRads);
	}

	public Angle2d times(double multiplier) {
		return new Angle2d(valueRads * multiplier);
	}

	public Angle2d div(double denominator) {
		return times(1.0 / denominator);
	}

	public Angle2d rotateBy(Angle2d other) {
		return Angle2d.fromRadians(valueRads + other.valueRads);
	}

	public Angle2d relativeTo(Angle2d other) {
		return rotateBy(other.unaryMinus());
	}

	// use carefully, it does a Modulo 360
	public Rotation2d toRotation2d() {
		return Rotation2d.fromRadians(valueRads);
	}

	public Angle getMeasure() {
		return Radians.of(valueRads);
	}

	public double getRadians() {
		return valueRads;
	}

	public double getDegrees() {
		return Math.toDegrees(valueRads);
	}

	public double getRotations() {
		return Units.radiansToRotations(valueRads);
	}

	public double getCos() {
		return cos;
	}

	public double getSin() {
		return sin;
	}

	public double getTan() {
		return sin / cos;
	}

	public Angle2d moduloOneRotation() {
		return fromRadians(valueRads % (Math.PI * 2));
	}

	@Override
	public String toString() {
		return String.format("Angle2d(Rads: %.2f, Deg: %.2f)", valueRads, Math.toDegrees(valueRads));
	}

	@Override
	public boolean equals(Object obj) {
		return obj instanceof Angle2d other && Math.hypot(cos - other.cos, sin - other.sin) < 1E-9;
	}

	@Override
	public int hashCode() {
		return Objects.hash(valueRads);
	}

	@Override
	public Angle2d interpolate(Angle2d endValue, double interpolationFraction) {
		return plus(endValue.minus(this).times(Math.max(0, Math.min(interpolationFraction, 1))));
	}

}
