package frc.utils;

import frc.utils.costomUnits.Angle2d;
import org.wpilib.math.geometry.Rotation3d;
import java.util.function.DoubleFunction;

public enum AngleUnit {

	ROTATIONS(Angle2d::fromRotations),
	RADIANS(Angle2d::fromRadians),
	DEGREES(Angle2d::fromDegrees);

	private final DoubleFunction<Angle2d> toAngle2d;
	private final TriFunction<Double, Double, Double, Rotation3d> toRotation3d;

	AngleUnit(DoubleFunction<Angle2d> toAngle2d) {
		this.toAngle2d = toAngle2d;
		this.toRotation3d = (
			roll,
			pitch,
			yaw
		) -> new Rotation3d(toAngle2d.apply(roll).getRadians(), toAngle2d.apply(pitch).getRadians(), toAngle2d.apply(yaw).getRadians());
	}

	public Angle2d toAngle2d(double value) {
		return toAngle2d.apply(value);
	}

	public Rotation3d toRotation3d(double roll, double pitch, double yaw) {
		return toRotation3d.apply(roll, pitch, yaw);
	}

}
