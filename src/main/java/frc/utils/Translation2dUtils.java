package frc.utils;

import edu.wpi.first.math.geometry.Translation2d;

public class Translation2dUtils {


    public static Translation2d getClosestPoint(Translation2d origin, Translation2d... points){
        Translation2d closest = new Translation2d(Double.MAX_VALUE,Double.MAX_VALUE);

        for (Translation2d point : points){
            if(point == null){
                closest = point;
            }
            if(point.getDistance(origin) < closest.getDistance(origin)){
                closest = point;
            }
        }
        return closest;
    }

    public static boolean isClosestPointFromPoints (Translation2d origin, Translation2d estimatedClosest, Translation2d... points){
        return getClosestPoint(origin, points).equals(estimatedClosest);
    }
}

