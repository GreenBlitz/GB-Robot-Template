package frc.utils.pose;

import edu.wpi.first.math.geometry.*;
import frc.robot.hardware.signal.TimedValue;
import frc.robot.poseestimator.Pose2dComponentsValue;
import frc.robot.poseestimator.Pose3dComponentsValue;
import frc.robot.vision.data.VisionData;
import frc.utils.AngleUnit;
import frc.utils.alerts.Alert;

public class PoseUtils {

	public static Pose3d poseArrayToPose3D(double[] poseArray, AngleUnit angleUnit) {
		if (poseArray.length != Pose3dComponentsValue.POSE3D_COMPONENTS_AMOUNT) {
			new Alert(Alert.AlertType.WARNING, "gotBadPoseArray");
		}
		return new Pose3d(
			new Translation3d(
				poseArray[Pose3dComponentsValue.X_VALUE.getIndex()],
				poseArray[Pose3dComponentsValue.Y_VALUE.getIndex()],
				poseArray[Pose3dComponentsValue.Z_VALUE.getIndex()]
			),
			new Rotation3d(
				angleUnit.toRotation2d(poseArray[Pose3dComponentsValue.ROLL_VALUE.getIndex()]).getRadians(),
				angleUnit.toRotation2d(poseArray[Pose3dComponentsValue.PITCH_VALUE.getIndex()]).getRadians(),
				angleUnit.toRotation2d(poseArray[Pose3dComponentsValue.YAW_VALUE.getIndex()]).getRadians()
			)
		);
	}

	public static Pose2d poseArrayToPose2D(double[] poseArray, AngleUnit angleUnit) {
		if (poseArray.length != Pose2dComponentsValue.POSE2D_COMPONENTS_AMOUNT) {
			new Alert(Alert.AlertType.WARNING, "gotBadPoseArray");
		}
		return new Pose2d(
			poseArray[Pose2dComponentsValue.X_VALUE.getIndex()],
			poseArray[Pose2dComponentsValue.X_VALUE.getIndex()],
			angleUnit.toRotation2d(poseArray[Pose2dComponentsValue.ROTATION_VALUE.getIndex()])
		);
	}

	public static TimedValue<Rotation2d> VisionDataToHeadingData(VisionData visionData) {
		return new TimedValue<Rotation2d>(visionData.getEstimatedPose().getRotation().toRotation2d(), visionData.getTimestamp());
	}

}
