package frc.robot.vision.photonvision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;

public class PhotonVisionConstants {

	public static final AprilTagFieldLayout APRIL_TAG_FIELD_LAYOUT = AprilTagFields.kDefaultField.loadAprilTagLayoutField();

	public static final CameraConfiguration[] CAMERAS_CONFIGURATION = {
		new CameraConfiguration(
			"camera1",
			PhotonVisionTarget.APRIL_TAG,
			new Transform3d(0.16, -0.33 - 0.04, 0.65 + 0.09, new Rotation3d(20, 90, 0))
		), // Arducam_OV9281_USB_Camera (1)
	};

	public static final String camerasLogPathPrefix = "PhotonCameras/";

	public static final double MAXIMUM_ALLOWED_LATENCY = 0.2;

	public static final double MAXIMUM_ALLOWED_AMBIGUITY = 0.2;

	public static final double AMBIGUITY_TO_LOCATION_STANDARD_DEVIATIONS_FACTOR = 2; // ! Shall be calibrated

	public static final double AMBIGUITY_TO_ROTATION_STANDARD_DEVIATIONS_FACTOR = 2; // ! Shall be calibrated

	public static final double APRIL_TAG_HEIGHT_TOLERANCE = 1; // ! Shall be calibrated

}
