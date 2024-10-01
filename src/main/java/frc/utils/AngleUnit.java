package frc.utils;

import edu.wpi.first.math.geometry.Rotation2d;

import java.util.function.DoubleFunction;

public enum AngleUnit {

	ROTATIONS(Rotation2d::fromRotations),
	RADIANS(Rotation2d::fromRadians),
	DEGREES(Rotation2d::fromDegrees);

	private final DoubleFunction<Rotation2d> toRotation2d;

	AngleUnit(DoubleFunction<Rotation2d> toRotation2d) {
		this.toRotation2d = toRotation2d;
	}

	public Rotation2d toAngle(double value) {
		return toRotation2d.apply(value);
	}

}
