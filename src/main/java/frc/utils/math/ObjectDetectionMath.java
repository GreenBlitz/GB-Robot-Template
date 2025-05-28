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
		pitchAndYaw.rotateAround(new Translation2d(), Rotation2d.fromRadians(-roll));
		return pitchAndYaw;
	}

	public static double sideTrigCalculation(Rotation2d pitchFromCamMiddle, Pose3d cameraPose) {
		Rotation2d pitch = Rotation2d.fromRadians(Math.abs(cameraPose.getRotation().getY() + pitchFromCamMiddle.getRadians()));
	    double cameraHeightMeters = cameraPose.getZ();
        return (Math.tan(pitch.getRadians()) * cameraHeightMeters);
    }

    public static double airviewTrigCalculation(Rotation2d yawFromCamMiddle, Pose3d cameraPose, double distanceMeters) {
        Rotation2d yaw = Rotation2d.fromRadians(Math.abs(cameraPose.getRotation().getZ() + yawFromCamMiddle.getRadians()));
        return (Math.tan(yaw.getRadians()) * distanceMeters);
    }

}
