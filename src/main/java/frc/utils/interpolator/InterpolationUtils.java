package frc.utils.interpolator;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Translation2d;
import frc.utils.Translation2dUtils;

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

    @SafeVarargs
    public static boolean doesEveryPointsExist(Pair<Translation2d, Double>... points) {
        for (Pair<Translation2d, Double> point : points) {
            if (point == null || point.getFirst() == null) {
                return false;
            }
        }
        return true;
    }

    @SafeVarargs
    public static boolean isQueryPointInBoundingBox(Translation2d query, Pair<Translation2d, Double>... knownPoints) {
        double xMin = Double.MAX_VALUE, xMax = -Double.MAX_VALUE;
        double yMin = Double.MAX_VALUE, yMax = -Double.MAX_VALUE;

        for (Pair<Translation2d, Double> point : knownPoints) {
            xMin = Math.min(xMin, point.getFirst().getX());
            xMax = Math.max(xMax, point.getFirst().getX());
            yMin = Math.min(yMin, point.getFirst().getY());
            yMax = Math.max(yMax, point.getFirst().getY());
        }
        return query.getX() >= xMin && query.getX() <= xMax && query.getY() >= yMin && query.getY() <= yMax;
    }

    @SafeVarargs
    public static Pair<Translation2d, Double>[] createBoundingBox(Translation2d query, Pair<Translation2d, Double>... knownPoints) {
        Pair<Translation2d, Double> bottomLeft = null, bottomRight = null, topLeft = null, topRight = null;

        for (Pair<Translation2d, Double> point : knownPoints) {
            double x = point.getFirst().getX();
            double y = point.getFirst().getY();
            if (x <= query.getX() && y <= query.getY()) {
                if (
                        bottomLeft == null
                                || (x >= bottomLeft.getFirst().getX() && y >= bottomLeft.getFirst().getY())
                                && Translation2dUtils
                                .isClosestPointFromPoints(query, point.getFirst(), new Translation2d[]{bottomLeft.getFirst(), point.getFirst()})
                ) {
                    bottomLeft = point;
                }
            }
            if (x >= query.getX() && y <= query.getY()) {
                if (
                        bottomRight == null
                                || (x >= bottomRight.getFirst().getX() && y <= bottomRight.getFirst().getY())
                                && Translation2dUtils.isClosestPointFromPoints(
                                query,
                                point.getFirst(),
                                new Translation2d[]{bottomRight.getFirst(), point.getFirst()}
                        )
                ) {
                    bottomRight = point;
                }
            }
            if (x <= query.getX() && y >= query.getY()) {
                if (
                        topLeft == null
                                || (x >= topLeft.getFirst().getX() && y <= topLeft.getFirst().getY())
                                && Translation2dUtils.isClosestPointFromPoints(
                                query,
                                point.getFirst(),
                                new Translation2d[]{topLeft.getFirst(), point.getFirst()})
                ) {
                    topLeft = point;
                }
            }
            if (x >= query.getX() && y >= query.getY()) {
                if (
                        topRight == null
                                || (x <= topRight.getFirst().getX() && y <= topRight.getFirst().getY())
                                && Translation2dUtils
                                .isClosestPointFromPoints(
                                        query,
                                        point.getFirst(),
                                        new Translation2d[]{topRight.getFirst(), point.getFirst()}
                                )
                ) {
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
        double x0 = bottomLeft.getFirst().getX(), y0 = bottomLeft.getFirst().getY();
        double x1 = bottomRight.getFirst().getX(), y1 = topLeft.getFirst().getY();

        double x = query.getX();
        double y = query.getY();


        return ((bottomLeft.getSecond() * (x1 - x) * (y1 - y)
                + bottomRight.getSecond() * (x - x0) * (y1 - y)
                + topLeft.getSecond() * (x1 - x) * (y - y0)
                + topRight.getSecond() * (x - x0) * (y - y0))) / ((x1 - x0) * (y1 - y0));
    }

    @SafeVarargs
    public static double bilinearInterpolate(Translation2d query, Pair<Translation2d, Double>... knownPoints) throws Exception {
        if (knownPoints.length < 4) {
            throw new IllegalArgumentException("At least 4 points and values are required.");
        }

        if (!isQueryPointInBoundingBox(query, knownPoints)) {
            throw new IllegalArgumentException("Query point is outside the bounding rectangle of the given points.");
        }

        Pair<Translation2d, Double>[] boundingBox = createBoundingBox(query, knownPoints);
        Pair<Translation2d, Double> bottomLeft = boundingBox[0];
        Pair<Translation2d, Double> bottomRight = boundingBox[1];
        Pair<Translation2d, Double> topLeft = boundingBox[2];
        Pair<Translation2d, Double> topRight = boundingBox[3];

        if (!doesEveryPointsExist(bottomLeft, bottomRight, topLeft, topRight)) {
            throw new IllegalStateException("Bounding rectangle is not properly formed by given points.");
        }
        return biLinearInterpolate(bottomLeft, bottomRight, topLeft, topRight, query);
    }

}
