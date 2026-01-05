package frc.utils.pose;

import edu.wpi.first.math.geometry.Translation2d;

public class swerveUtil {
    public static Translation2d getMajority(Translation2d[] arr){
        return arr[0].equals(arr[1])|| arr[0].equals(arr[2]) ? arr[0]:arr[1]; //add tolerance
    }
}
