package frc.utils.interpolator;

import edu.wpi.first.math.geometry.Translation2d;

public class InterpolationUtils {

    /**
     *
     * @param neighbor1 known point 1
     * @param value1 value of known point 1
     * @param neighbor2 known point 2
     * @param value2 value of known point 2
     * @param interpolatingTargetPoint the UN-KNOWN point wanted value
     *
     * @return the interpolated value of the interpolatingTargetPoint
     * */
	public static double linearInterpolation2d(
		Translation2d neighbor1,
		double value1,
		Translation2d neighbor2,
		double value2,
		Translation2d interpolatingTargetPoint
	) {
		double distanceTargetToNeighbor1 = interpolatingTargetPoint.getDistance(neighbor1);
		double distanceTargetToNeighbor2 = interpolatingTargetPoint.getDistance(neighbor2);

		double totalDistance = distanceTargetToNeighbor1 + distanceTargetToNeighbor2;
		double slope = (value2 - value1) / (totalDistance);
		return value1 + slope * distanceTargetToNeighbor1;
	}

}
