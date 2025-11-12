package frc.robot.subsystems.doubleInterpolation;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import frc.utils.InterpolationMap;

import java.util.Map;
import java.util.function.Supplier;

public class TemporaryInterpolation {

	public static final InterpolationMap<Double, Rotation2d> HOOD_INTERPOLATION_MAP = new InterpolationMap<Double, Rotation2d>(
		InverseInterpolator.forDouble(),
		InterpolationMap.interpolatorForRotation2d(),
		Map.of(
			1.0,
			Rotation2d.fromDegrees(75),
			2.0,
			Rotation2d.fromDegrees(68),
			3.0,
			Rotation2d.fromDegrees(57),
			4.0,
			Rotation2d.fromDegrees(52),
			5.0,
			Rotation2d.fromDegrees(49)
		)
	);

	public static final InterpolationMap<Double, Rotation2d> FLYWHEEL_INTERPOLATION_MAP = new InterpolationMap<Double, Rotation2d>(
		InverseInterpolator.forDouble(),
		InterpolationMap.interpolatorForRotation2d(),
		Map.of(
			1.0,
			Rotation2d.fromRotations(5),
			2.0,
			Rotation2d.fromRotations(7),
			3.0,
			Rotation2d.fromRotations(10),
			4.0,
			Rotation2d.fromRotations(13),
			5.0,
			Rotation2d.fromRotations(16)
		)
	);

	public static Rotation2d hoodInterpolation(Supplier<Double> distanceFromTower) {
		return HOOD_INTERPOLATION_MAP.get(distanceFromTower.get());
	}

	public static Rotation2d flywheelInterpolation(Supplier<Double> distanceFromTower) {
		return FLYWHEEL_INTERPOLATION_MAP.get(distanceFromTower.get());
	}

}
