package frc.utils;

import edu.wpi.first.math.geometry.Translation2d;

import java.util.Objects;

public class Translation2dUtils {


    public static Translation2d findClosestPointAfter(Translation2d target, Translation2d[] knownPoints) {
        Translation2d closestPoint = null;
        double minDistance = Double.MAX_VALUE;

        for (Translation2d point : knownPoints) {
            if (isAfter(target, point)) {
                double distance = point.getDistance(target);
                if (distance < minDistance) {
                    minDistance = distance;
                    closestPoint = point;
                }
            }
        }
        System.out.println(closestPoint);
        return closestPoint;
    }

    public static Translation2d findClosestPointBefore(Translation2d target, Translation2d[] knownPoints) {
        Translation2d closestPoint = null;
        double minDistance = Double.MAX_VALUE;

        for (Translation2d point : knownPoints) {
            if (!isAfter(target, point)) {
                double distance = point.getDistance(target);
                if (distance < minDistance) {
                    minDistance = distance;
                    closestPoint = point;
                }
            }
        }

        return closestPoint;
    }

	public static boolean isAfter(Translation2d comparisonPoint, Translation2d point) {
		return comparisonPoint.getX() <= point.getX() && comparisonPoint.getY() <= point.getY();
	}
}
