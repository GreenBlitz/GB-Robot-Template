package frc.utils.interpolator;

import edu.wpi.first.math.geometry.Translation2d;

import java.util.Arrays;

public class BilinearInterpolation {

    public static void main(String[] args) {
        // Example usage
        Translation2d[] points = {
                new Translation2d(0,0),
                new Translation2d(1,0),
                new Translation2d(0,1),
                new Translation2d(1,1)
        };
        double[] values = {1,2,2,3 };
        Translation2d query = new Translation2d(0.5,0.5);

        double interpolatedValue = bilinearInterpolate(points, values, query);
        System.out.println("Interpolated Value: " + interpolatedValue);
    }

    public static double bilinearInterpolate(Translation2d[] points, double[] values, Translation2d query) {
        if (points.length < 4 || values.length < 4) {
            throw new IllegalArgumentException("At least 4 points and values are required.");
        }

        double xMin = Double.MAX_VALUE, xMax = -Double.MAX_VALUE;
        double yMin = Double.MAX_VALUE, yMax = -Double.MAX_VALUE;

        for (Translation2d point : points) {
            xMin = Math.min(xMin, point.getX());
            xMax = Math.max(xMax, point.getX());
            yMin = Math.min(yMin, point.getY());
            yMax = Math.max(yMax, point.getY());
        }

        if (query.getX() < xMin || query.getX() > xMax || query.getY() < yMin || query.getY() > yMax) {
            throw new IllegalArgumentException("Query point is outside the bounding rectangle of the given points.");
        }

        Translation2d bottomLeft = null, bottomRight = null, topLeft = null, topRight = null;
        double bottomLeftValue = Double.NaN, bottomRightValue = Double.NaN, topLeftValue = Double.NaN, topRightValue = Double.NaN;
        for (int i = 0; i < points.length; i++) {
            Translation2d point = points[i];
            double x = point.getX();
            double y = point.getY();
            if (x <= query.getX() && y <= query.getY()) {
                if (bottomLeft == null || x > bottomLeft.getX() || y > bottomLeft.getY()) {
                    bottomLeft = point;
                    bottomLeftValue = values[i];
                }
            }
            if (x >= query.getX() && y <= query.getY()) {
                if (bottomRight == null || x < bottomRight.getX() || y > bottomRight.getY()) {
                    bottomRight = point;
                    bottomRightValue = values[i];
                }
            }
            if (x <= query.getX() && y >= query.getY()) {
                if (topLeft == null || x > topLeft.getX() || y < topLeft.getY()) {
                    topLeft = point;
                    topLeftValue = values[i];
                }
            }
            if (x >= query.getX() && y >= query.getY()) {
                if (topRight == null || x < topRight.getX() || y < topRight.getY()) {
                    topRight = point;
                    topRightValue = values[i];
                }
            }
        }

        if (bottomLeft == null || bottomRight == null || topLeft == null || topRight == null) {
            throw new IllegalStateException("Bounding rectangle is not properly formed by given points.");
        }

        // Retrieve coordinates for the corners
        double x1 = bottomLeft.getX(), y1 = bottomLeft.getY();
        double x2 = bottomRight.getX(), y2 = topLeft.getY();

        // Perform bilinear interpolation
        double x = query.getX();
        double y = query.getY();

        double result = (bottomLeftValue * (x2 - x) * (y2 - y) +
                bottomRightValue * (x - x1) * (y2 - y) +
                topLeftValue * (x2 - x) * (y - y1) +
                topRightValue * (x - x1) * (y - y1)) / ((x2 - x1) * (y2 - y1));

        return result;
    }
}