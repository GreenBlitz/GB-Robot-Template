package frc.robot.turret;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class TurretUtils {
    public static Rotation2d calculateAbsoluteTargetAngle (Translation2d robotPose, Translation2d targetPose){
        Translation2d normalizedTargetPosition = targetPose.minus(robotPose);
        return new Rotation2d(
                normalizedTargetPosition.getX(),
                normalizedTargetPosition.getY()
        );
    }
}
