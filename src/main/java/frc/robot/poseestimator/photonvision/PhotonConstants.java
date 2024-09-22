package frc.robot.poseestimator.photonvision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Transform3d;


public class PhotonConstants {

	public static final AprilTagFieldLayout APRIL_TAG_FIELD_LAYOUT = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
	
	public static final CameraConfiguration[] CAMERAS_CONFIGURATION = {
		new CameraConfiguration(PhotonTarget.AprilTag, "camera1", new Transform3d()),
		new CameraConfiguration(PhotonTarget.AprilTag, "camera2", new Transform3d()),
	};
	
	public static final double MAXIMUM_LATENCY = 0.2;
	
	public static final double MAXIMUM_AMBIGUOUS = 0.2;

}
