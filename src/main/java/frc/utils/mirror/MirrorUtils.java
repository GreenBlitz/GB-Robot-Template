package frc.utils.mirror;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.constants.Field;
import frc.robot.constants.MathConstants;

public class MirrorUtils {

    public static Rotation2d getMirroredAngle(Rotation2d angle) {
        return MathConstants.HALF_CIRCLE.minus(angle);
    }

    public static double getMirroredX(double x){
        return Field.LENGTH_METERS - x;
    }

    public static double getMirroredY(double y){
        return Field.WIDTH_METERS - y;
    }

}
