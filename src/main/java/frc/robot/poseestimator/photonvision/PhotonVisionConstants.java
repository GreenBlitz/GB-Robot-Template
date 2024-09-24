package frc.robot.poseestimator.photonvision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Transform3d;

public class PhotonVisionConstants {

	public static final AprilTagFieldLayout APRIL_TAG_FIELD_LAYOUT = AprilTagFields.kDefaultField.loadAprilTagLayoutField();

	public static final CameraConfiguration[] CAMERAS_CONFIGURATION = {
		new CameraConfiguration("camera1", PhotonVisionTarget.APRIL_TAG, new Transform3d()), // Arducam_OV9281_USB_Camera (1)
	};

	public static final String camerasLogPathPrefix = "PhotonCameras/";

	public static final double MAXIMUM_ALLOWED_LATENCY = 0.2;

	public static final double MAXIMUM_ALLOWED_AMBIGUITY = 0.2;

	public static final double AMBIGUITY_TO_LOCATION_STANDARD_DEVIATIONS_FACTOR = 2; // ! Shall be calibrated

	public static final double AMBIGUITY_TO_ROTATION_STANDARD_DEVIATIONS_FACTOR = 2; // ! Shall be calibrated

	public static final double APRIL_TAG_MINIMUM_HEIGHT = 1; // ! Shall be calibrated

	public static final double APRIL_TAG_MAXIMUM_HEIGHT = 3; // ! Shall be calibrated

}
