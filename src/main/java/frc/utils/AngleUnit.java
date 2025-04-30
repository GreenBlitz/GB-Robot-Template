package frc.utils;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import java.util.function.DoubleFunction;

public enum AngleUnit {

	ROTATIONS(Rotation2d::fromRotations),
	RADIANS(Rotation2d::fromRadians),
	DEGREES(Rotation2d::fromDegrees);

	private final DoubleFunction<Rotation2d> toRotation2d;
	private final TriFunction<Double, Double, Double, Rotation3d> toRotation3d;

	AngleUnit(DoubleFunction<Rotation2d> toRotation2d) {
		this.toRotation2d = toRotation2d;
		this.toRotation3d = (
			roll,
			pitch,
			yaw
		) -> new Rotation3d(toRotation2d.apply(roll).getRadians(), toRotation2d.apply(pitch).getRadians(), toRotation2d.apply(yaw).getRadians());
	}

	public Rotation2d toRotation2d(double value) {
		return toRotation2d.apply(value);
	}

	public Rotation3d toRotation3d(double roll, double pitch, double yaw) {
		return toRotation3d.apply(roll, pitch, yaw);
	}

}
