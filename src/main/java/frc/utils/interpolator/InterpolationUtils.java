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

	static double interpolate(Translation2d neighbor1, double value1, Translation2d neighbor2, double value2, Translation2d interpolatingTargetPoint) {
		if (neighbor1.getX() == neighbor2.getX()) {
			if (interpolatingTargetPoint.getX() != neighbor1.getX()){
				throw new IllegalArgumentException("Target x-coordinate must be the same as x1 and x2.");
			}
			return value1 + (interpolatingTargetPoint.getY() - neighbor1.getY()) * (value2 - value1) / (neighbor2.getY() - neighbor1.getY());
		}
		double interpolationFactor = (interpolatingTargetPoint.getX() - neighbor1.getX()) / (neighbor2.getX() - neighbor1.getX());
		return value1 + interpolationFactor * (value2 - value1);
	}
}
