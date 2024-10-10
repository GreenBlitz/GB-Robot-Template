package frc.robot.subsystems.pivot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.Interpolator;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.math.util.Units;

public class DistanceToAngleMap {

	public static InterpolatingTreeMap<
		Double,
		Double> DISTANCE_TO_ANGLE = new InterpolatingTreeMap<>(InverseInterpolator.forDouble(), Interpolator.forDouble());

	static {
		DISTANCE_TO_ANGLE.put(0.9, Units.degreesToRotations(58));
		DISTANCE_TO_ANGLE.put(1.5, Units.degreesToRotations(48.5));
		DISTANCE_TO_ANGLE.put(2.0, Units.degreesToRotations(41));
		DISTANCE_TO_ANGLE.put(2.65, Units.degreesToRotations(38.2));
		DISTANCE_TO_ANGLE.put(2.7, Units.degreesToRotations(34.2));
		DISTANCE_TO_ANGLE.put(3.25, Units.degreesToRotations(33.75));
		DISTANCE_TO_ANGLE.put(3.47, Units.degreesToRotations(32.3));
		DISTANCE_TO_ANGLE.put(3.92, Units.degreesToRotations(29.1));
		DISTANCE_TO_ANGLE.put(4.15, Units.degreesToRotations(28.9));
		DISTANCE_TO_ANGLE.put(4.5, Units.degreesToRotations(27.6));
		DISTANCE_TO_ANGLE.put(5.0, Units.degreesToRotations(26.2));
		DISTANCE_TO_ANGLE.put(5.5, Units.degreesToRotations(25));
	}

	public static Rotation2d getAngle(double distanceInMeters) {
		return Rotation2d.fromRotations(DISTANCE_TO_ANGLE.get(distanceInMeters));
	}

}

