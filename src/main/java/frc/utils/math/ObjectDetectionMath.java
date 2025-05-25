package frc.utils.math;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class ObjectDetectionMath {

    public static Translation2d pixelsToPitchAndYaw(Translation2d pose) {
        double yaw = pose.getX();
        double pitch = pose.getY();
        return new Translation2d(yaw, pitch);
    }

    public static Translation2d undoRoll(Translation2d pitchAndYaw, Pose3d cameraPose) {
        double roll = cameraPose.getRotation().getX();
        pitchAndYaw.rotateAround(new Translation2d(), Rotation2d.fromRadians(roll));
        return pitchAndYaw;
    }

}
