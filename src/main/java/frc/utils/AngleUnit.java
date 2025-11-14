package frc.utils;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import java.util.function.DoubleFunction;
import java.util.function.Function;

public enum AngleUnit {

	ROTATIONS(Rotation2d::fromRotations, Rotation2d::getRotations),
	RADIANS(Rotation2d::fromRadians, Rotation2d::getRadians),
	DEGREES(Rotation2d::fromDegrees, Rotation2d::getDegrees);

	private final DoubleFunction<Rotation2d> toRotation2d;
	private final Function<Rotation2d, Double> fromRotation2d;
	private final TriFunction<Double, Double, Double, Rotation3d> toRotation3d;

	AngleUnit(DoubleFunction<Rotation2d> toRotation2d, Function<Rotation2d, Double> fromRotation2d) {
		this.toRotation2d = toRotation2d;
		this.fromRotation2d = fromRotation2d;
		this.toRotation3d = (
			roll,
			pitch,
			yaw
		) -> new Rotation3d(toRotation2d.apply(roll).getRadians(), toRotation2d.apply(pitch).getRadians(), toRotation2d.apply(yaw).getRadians());
	}

	public Rotation2d toRotation2d(double value) {
		return toRotation2d.apply(value);
	}

	public double fromRotation2d(Rotation2d rotation) {
		return fromRotation2d.apply(rotation);
	}

	public Rotation3d toRotation3d(double roll, double pitch, double yaw) {
		return toRotation3d.apply(roll, pitch, yaw);
	}

}
