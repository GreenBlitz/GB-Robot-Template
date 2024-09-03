package frc.utils;

import edu.wpi.first.math.geometry.Translation2d;

import java.util.Objects;

public class Translation2dUtils {

	public static Translation2d getPositiveClosest(Translation2d comparePoint, Translation2d... knownPoints) {
		Translation2d closestPositivePoint = null;

		for (Translation2d knownPoint : knownPoints) {
			if (isAfterTranslation(comparePoint, knownPoint)) {
				if (Objects.isNull(closestPositivePoint)) {
					closestPositivePoint = knownPoint;
				}
				if (comparePoint.getDistance(knownPoint) < comparePoint.getDistance(closestPositivePoint)) {
					closestPositivePoint = knownPoint;
				}
			}
		}
		return closestPositivePoint;
	}

    public static Translation2d getNegativeClosest(Translation2d comparePoint, Translation2d... knownPoints) {
        Translation2d closestNegativePoint = null;

        for (Translation2d knownPoint : knownPoints) {
            if (!isAfterTranslation(comparePoint, knownPoint)) {
                if (Objects.isNull(closestNegativePoint)) {
                    closestNegativePoint = knownPoint;
                }
                if (comparePoint.getDistance(knownPoint) < comparePoint.getDistance(closestNegativePoint)) {
                    closestNegativePoint = knownPoint;
                }
            }
        }
        return closestNegativePoint;
    }



	public static boolean isAfterTranslation(Translation2d comparisonPoint, Translation2d point) {
		return comparisonPoint.getX() <= point.getX() && comparisonPoint.getY() <= point.getY();
	}
}
