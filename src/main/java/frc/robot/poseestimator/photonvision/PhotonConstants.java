package frc.robot.poseestimator.photonvision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;


public class PhotonConstants {

	public static final AprilTagFieldLayout APRIL_TAG_FIELD_LAYOUT = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
	public static final CameraConfiguration[] camerasConfig = {
		new CameraConfiguration(PhotonTarget.AprilTag, "camera1"),
		new CameraConfiguration(PhotonTarget.AprilTag, "camera2"),};

}
