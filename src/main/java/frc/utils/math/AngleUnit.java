package frc.utils.math;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import frc.utils.TriFunction;

import java.util.function.DoubleFunction;

public enum AngleUnit {

	ROTATIONS(
		Rotation2d::fromRotations,
		(roll, pitch, yaw) -> new Rotation3d(
			Rotation2d.fromRotations(roll).getRadians(),
			Rotation2d.fromRotations(pitch).getRadians(),
			Rotation2d.fromRotations(yaw).getRadians()
		)
	),
	RADIANS(Rotation2d::fromRadians, Rotation3d::new),
	DEGREES(
		Rotation2d::fromDegrees,
		(roll, pitch, yaw) -> new Rotation3d(
			Rotation2d.fromDegrees(roll).getRadians(),
			Rotation2d.fromDegrees(pitch).getRadians(),
			Rotation2d.fromDegrees(yaw).getRadians()
		)
	);

	private final DoubleFunction<Rotation2d> toRotation2d;
	private final TriFunction<Double, Double, Double, Rotation3d> toRotation3d;

	AngleUnit(DoubleFunction<Rotation2d> toRotation2d, TriFunction<Double, Double, Double, Rotation3d> toRotation3d) {
		this.toRotation2d = toRotation2d;
		this.toRotation3d = toRotation3d;
	}

	public Rotation2d toRotation2d(double value) {
		return toRotation2d.apply(value);
	}

	public Rotation3d toRotation3d(double roll, double pitch, double yaw) {
		return toRotation3d.apply(roll, pitch, yaw);
	}

}
