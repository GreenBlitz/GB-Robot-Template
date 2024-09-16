package frc.utils.interpolator;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Translation2d;

public class InterpolationUtils {

    /**
     * @param neighbor1                known point 1
     * @param value1                   value of known point 1
     * @param neighbor2                known point 2
     * @param value2                   value of known point 2
     * @param interpolatingTargetPoint the UN-KNOWN point wanted value
     * @return the interpolated value of the interpolatingTargetPoint
     */

    static double
    interpolate(Translation2d neighbor1, double value1, Translation2d neighbor2, double value2, Translation2d interpolatingTargetPoint) {
        if (neighbor1.getX() == neighbor2.getX()) {
            return value1 + (interpolatingTargetPoint.getY() - neighbor1.getY()) * (value2 - value1) / (neighbor2.getY() - neighbor1.getY());
        }
        double interpolationFactor = (interpolatingTargetPoint.getX() - neighbor1.getX()) / (neighbor2.getX() - neighbor1.getX());
        return value1 + interpolationFactor * (value2 - value1);
    }

    public static boolean isAllPointsExists(Translation2d... points) {
        for (Translation2d point : points) {
            if (point == null) {
                return false;
            }
        }
        return true;
    }

    @SafeVarargs
    public static boolean isAllPointsExists(Pair<Translation2d, Double>... points) {
        for (Pair<Translation2d, Double> point : points) {
            if (point == null || isAllPointsExists(point.getFirst())) {
                return false;
            }
        }
        return true;
    }

    public static boolean isQueryPointInBoundingBox(Translation2d query, Pair<Translation2d, Double>... knownPoints) {
        double xMin = Double.MAX_VALUE, xMax = -Double.MAX_VALUE;
        double yMin = Double.MAX_VALUE, yMax = -Double.MAX_VALUE;

        for (Pair<Translation2d, Double> point : knownPoints) {
            xMin = Math.min(xMin, point.getFirst().getX());
            xMax = Math.max(xMax, point.getFirst().getX());
            yMin = Math.min(yMin, point.getFirst().getY());
            yMax = Math.max(yMax, point.getFirst().getY());
        }
        return query.getX() > xMin || query.getX() < xMax || query.getY() > yMin || query.getY() < yMax;
    }

    @SafeVarargs
    public static Pair<Translation2d, Double>[] createBoundingBox(Translation2d query, Pair<Translation2d, Double>... knownPoints) {
        Pair<Translation2d, Double> bottomLeft = null, bottomRight = null, topLeft = null, topRight = null;

        for (Pair<Translation2d, Double> point : knownPoints) {
            double x = point.getFirst().getX();
            double y = point.getFirst().getY();
            if (x <= query.getX() && y <= query.getY()) {
                if (bottomLeft == null || x > bottomLeft.getFirst().getX() || y > bottomLeft.getFirst().getY()) {
                    bottomLeft = point;
                }
            }
            if (x >= query.getX() && y <= query.getY()) {
                if (bottomRight == null || x < bottomRight.getFirst().getX() || y > bottomRight.getFirst().getY()) {
                    bottomRight = point;
                }
            }
            if (x <= query.getX() && y >= query.getY()) {
                if (topLeft == null || x > topLeft.getFirst().getX() || y < topLeft.getFirst().getY()) {
                    topLeft = point;
                }
            }
            if (x >= query.getX() && y >= query.getY()) {
                if (topRight == null || x < topRight.getFirst().getX() || y < topRight.getFirst().getY()) {
                    topRight = point;
                }
            }
        }
        return new Pair[]{bottomLeft, bottomRight, topLeft, topRight};
    }

    public static double biLinearInterpolate(
            Pair<Translation2d, Double> bottomLeft,
            Pair<Translation2d, Double> bottomRight,
            Pair<Translation2d, Double> topLeft,
            Pair<Translation2d, Double> topRight,
            Translation2d query
    ) {
        double x1 = bottomLeft.getFirst().getX(), y1 = bottomLeft.getFirst().getY();
        double x2 = bottomRight.getFirst().getX(), y2 = topLeft.getFirst().getY();

        double x = query.getX();
        double y = query.getY();

        return (bottomLeft.getSecond() * (x2 - x) * (y2 - y)
                + bottomRight.getSecond() * (x - x1) * (y2 - y)
                + topLeft.getSecond() * (x2 - x) * (y - y1)
                + topRight.getSecond() * (x - x1) * (y - y1)) / ((x2 - x1) * (y2 - y1));
    }

}
