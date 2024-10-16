package frc.robot.subsystems.pivot;

import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.Interpolator;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.math.util.Units;

public class PivotInterpolationMap {

	public static InterpolatingTreeMap<
		Double,
		Double> METERS_TO_RADIANS = new InterpolatingTreeMap<>(InverseInterpolator.forDouble(), Interpolator.forDouble());

	static {//perhaps it is shifted by 0.2 meters in othe fields, in steampunk it is 1.1, 1.7
		METERS_TO_RADIANS.put(1.1, Units.degreesToRadians(58.5));
		METERS_TO_RADIANS.put(1.7, Units.degreesToRadians(51));
		METERS_TO_RADIANS.put(2.2, Units.degreesToRadians(46));
		METERS_TO_RADIANS.put(2.7, Units.degreesToRadians(43));
		METERS_TO_RADIANS.put(3.0, Units.degreesToRadians(40));
		METERS_TO_RADIANS.put(3.3, Units.degreesToRadians(37));
		METERS_TO_RADIANS.put(3.6, Units.degreesToRadians(34));
		METERS_TO_RADIANS.put(3.9, Units.degreesToRadians(31));
		METERS_TO_RADIANS.put(4.2, Units.degreesToRadians(28));
		METERS_TO_RADIANS.put(4.5, Units.degreesToRadians(25));
	}

}
