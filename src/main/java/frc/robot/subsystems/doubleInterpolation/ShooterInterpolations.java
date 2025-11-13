package frc.robot.subsystems.doubleInterpolation;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import frc.utils.InterpolationMap;

import java.util.Map;
import java.util.function.Supplier;

public class ShooterInterpolations {

	public static final InterpolationMap<Double, Rotation2d> HOOD_INTERPOLATION_MAP = new InterpolationMap<Double, Rotation2d>(
		InverseInterpolator.forDouble(),
		InterpolationMap.interpolatorForRotation2d(),
		Map.of(
		)
	);

	public static final InterpolationMap<Double, Rotation2d> FLYWHEEL_INTERPOLATION_MAP = new InterpolationMap<Double, Rotation2d>(
		InverseInterpolator.forDouble(),
		InterpolationMap.interpolatorForRotation2d(),
		Map.of()
	);
a
	public static Rotation2d hoodInterpolation(Supplier<Double> distanceFromTower) {
		return HOOD_INTERPOLATION_MAP.get(distanceFromTower.get());
	}

	public static Rotation2d flywheelInterpolation(Supplier<Double> distanceFromTower) {
		return FLYWHEEL_INTERPOLATION_MAP.get(distanceFromTower.get());
	}

}
