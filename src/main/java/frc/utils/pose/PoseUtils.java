package frc.utils.pose;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import frc.robot.hardware.signal.TimedValue;
import frc.robot.poseestimator.Pose2dComponentsValue;
import frc.robot.poseestimator.Pose3dComponentsValue;
import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.vision.data.VisionData;
import frc.utils.AngleUnit;
import frc.utils.alerts.Alert;

public class PoseUtils {

	public static Pose3d poseArrayToPose3D(double[] poseArray, AngleUnit angleUnit) {
		int requiredAmount = Pose3dComponentsValue.POSE3D_COMPONENTS_AMOUNT;
		if (poseArray.length != requiredAmount) {
			new Alert(Alert.AlertType.WARNING, "gotBadPoseArrayWith" + poseArray.length + "ElementsInsteadOf" + requiredAmount);
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
		int requiredAmount = Pose2dComponentsValue.POSE2D_COMPONENTS_AMOUNT;
		if (poseArray.length != requiredAmount) {
			new Alert(Alert.AlertType.WARNING, "gotBadPoseArrayWith" + poseArray.length + "ElementsInsteadOf" + requiredAmount);
		}
		return new Pose2d(
			poseArray[Pose2dComponentsValue.X_VALUE.getIndex()],
			poseArray[Pose2dComponentsValue.X_VALUE.getIndex()],
			angleUnit.toRotation2d(poseArray[Pose2dComponentsValue.ROTATION_VALUE.getIndex()])
		);
	}

	public static double[] pose2DToPoseArray(Pose2d pose2d, AngleUnit angleUnit) {
		return new double[] {pose2d.getX(), pose2d.getY(), switch (angleUnit) {
			case RADIANS -> pose2d.getRotation().getRadians();
			case DEGREES -> pose2d.getRotation().getDegrees();
			case ROTATIONS -> pose2d.getRotation().getRotations();
		}};
	}

	public static double[] pose3DToPoseArray(Pose3d pose3d, AngleUnit angleUnit) {
		return switch (angleUnit) {
			case RADIANS ->
				new double[] {
					pose3d.getX(),
					pose3d.getY(),
					pose3d.getZ(),
					pose3d.getRotation().getX(),
					pose3d.getRotation().getY(),
					pose3d.getRotation().getZ()};
			case DEGREES ->
				new double[] {
					pose3d.getX(),
					pose3d.getY(),
					pose3d.getZ(),
					Rotation2d.fromRadians(pose3d.getRotation().getX()).getDegrees(),
					Rotation2d.fromRadians(pose3d.getRotation().getY()).getDegrees(),
					Rotation2d.fromRadians(pose3d.getRotation().getZ()).getDegrees()};
			case ROTATIONS ->
				new double[] {
					pose3d.getX(),
					pose3d.getY(),
					pose3d.getZ(),
					Rotation2d.fromRadians(pose3d.getRotation().getX()).getRotations(),
					Rotation2d.fromRadians(pose3d.getRotation().getY()).getRotations(),
					Rotation2d.fromRadians(pose3d.getRotation().getZ()).getRotations()};
		};
	}

	public static TimedValue<Rotation2d> visionDataToHeadingData(VisionData visionData) {
		return new TimedValue<>(visionData.getEstimatedPose().getRotation().toRotation2d(), visionData.getTimestamp());
	}

}
