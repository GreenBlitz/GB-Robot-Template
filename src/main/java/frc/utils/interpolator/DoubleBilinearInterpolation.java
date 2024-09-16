package frc.utils.interpolator;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Translation2d;

import java.util.LinkedList;
import java.util.stream.Stream;

public class DoubleBilinearInterpolation {

    public static void main(String[] args) {
        Pair<Translation2d,Double> p1 = new Pair<>(new Translation2d(0,0), 0.0);
        Pair<Translation2d,Double> p2 = new Pair<>(new Translation2d(1,1), 3.0);
        Pair<Translation2d,Double> p3 = new Pair<>(new Translation2d(1,0), 1.5);
        Pair<Translation2d,Double> p4 = new Pair<>(new Translation2d(0,1), 1.5);
        System.out.println(bilinearInterpolate(new Translation2d(0.5,0.5),p1, p2,p3,p4));
    }
    private final LinkedList<Pair<Translation2d, Double>> knowPoints;
    public DoubleBilinearInterpolation(){
        this.knowPoints = new LinkedList<>();
    }

    @SafeVarargs
    public static double bilinearInterpolate(Translation2d query, Pair<Translation2d, Double>... knownPoints) {
        if (knownPoints.length < 4) {
            throw new IllegalArgumentException("At least 4 points and values are required.");
        }

        double xMin = Double.MAX_VALUE, xMax = -Double.MAX_VALUE;
        double yMin = Double.MAX_VALUE, yMax = -Double.MAX_VALUE;

        for (Pair<Translation2d, Double> point : knownPoints) {
            xMin = Math.min(xMin, point.getFirst().getX());
            xMax = Math.max(xMax, point.getFirst().getX());
            yMin = Math.min(yMin, point.getFirst().getY());
            yMax = Math.max(yMax, point.getFirst().getY());
        }

        if (query.getX() < xMin || query.getX() > xMax || query.getY() < yMin || query.getY() > yMax) {
            throw new IllegalArgumentException("Query point is outside the bounding rectangle of the given points.");
        }

        Translation2d bottomLeft = null, bottomRight = null, topLeft = null, topRight = null;
        double bottomLeftValue = Double.NaN, bottomRightValue = Double.NaN, topLeftValue = Double.NaN, topRightValue = Double.NaN;

        for (Pair<Translation2d, Double> point : knownPoints){
            double x = point.getFirst().getX();
            double y = point.getFirst().getY();
            if (x <= query.getX() && y <= query.getY()) {
                if (bottomLeft == null || x > bottomLeft.getX() || y > bottomLeft.getY()) {
                    bottomLeft = point.getFirst();
                    bottomLeftValue = point.getSecond();
                }
            }
            if (x >= query.getX() && y <= query.getY()) {
                if (bottomRight == null || x < bottomRight.getX() || y > bottomRight.getY()) {
                    bottomRight = point.getFirst();
                    bottomRightValue = point.getSecond();
                }
            }
            if (x <= query.getX() && y >= query.getY()) {
                if (topLeft == null || x > topLeft.getX() || y < topLeft.getY()) {
                    topLeft = point.getFirst();
                    topLeftValue = point.getSecond();
                }
            }
            if (x >= query.getX() && y >= query.getY()) {
                if (topRight == null || x < topRight.getX() || y < topRight.getY()) {
                    topRight = point.getFirst();;
                    topRightValue = point.getSecond();
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

        return (bottomLeftValue * (x2 - x) * (y2 - y) +
                bottomRightValue * (x - x1) * (y2 - y) +
                topLeftValue * (x2 - x) * (y - y1) +
                topRightValue * (x - x1) * (y - y1)) / ((x2 - x1) * (y2 - y1));
    }
}