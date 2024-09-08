package frc.robot.hardware;

import edu.wpi.first.math.geometry.Rotation2d;

import java.util.function.DoubleFunction;

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
