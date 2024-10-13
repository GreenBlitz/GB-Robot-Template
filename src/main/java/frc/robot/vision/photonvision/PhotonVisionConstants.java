package frc.robot.vision.photonvision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.vision.visionposeestimations.VisionPoseEstimationFiltersTolerances;

public class PhotonVisionConstants {

	public static final AprilTagFieldLayout APRIL_TAG_FIELD_LAYOUT = AprilTagFields.kDefaultField.loadAprilTagLayoutField();

	public static final CameraConfiguration[] CAMERAS_CONFIGURATION = {
		new CameraConfiguration(
			"camera1",
			PhotonVisionTarget.ROBOT,
			new Transform3d(0.16, -0.33 - 0.04, 0.65 + 0.09, new Rotation3d(20, 90, 0))
		), // Arducam_OV9281_USB_Camera (1)
	};

	public static final String camerasLogPathPrefix = "PhotonCameras/";

	public static final VisionPoseEstimationFiltersTolerances DEFAULT_APRIL_TAGS_TOLERANCES = new VisionPoseEstimationFiltersTolerances(
		1.3,
		0.1,
		0.2,
		Rotation2d.fromDegrees(10),
		Rotation2d.fromDegrees(10),
		0.2,
		0.2,
		0.2
	);

}
