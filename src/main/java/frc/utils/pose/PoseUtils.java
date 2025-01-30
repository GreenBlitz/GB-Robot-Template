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
			angleUnit.toRotation3d(
				Pose3dComponentsValue.ROLL_VALUE.getIndex(),
				Pose3dComponentsValue.PITCH_VALUE.getIndex(),
				Pose3dComponentsValue.YAW_VALUE.getIndex()
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
		Rotation2d rotation = pose2d.getRotation();
		return new double[] {pose2d.getX(), pose2d.getY(), switch (angleUnit) {
			case RADIANS -> rotation.getRadians();
			case DEGREES -> rotation.getDegrees();
			case ROTATIONS -> rotation.getRotations();
		}};
	}

	public static double[] pose3DToPoseArray(Pose3d pose3d, AngleUnit angleUnit) {
		Rotation3d rotation = pose3d.getRotation();
		return switch (angleUnit) {
			case RADIANS -> new double[] {pose3d.getX(), pose3d.getY(), pose3d.getZ(), rotation.getX(), rotation.getY(), rotation.getZ()};
			case DEGREES ->
				new double[] {
					pose3d.getX(),
					pose3d.getY(),
					pose3d.getZ(),
					Rotation2d.fromRadians(rotation.getX()).getDegrees(),
					Rotation2d.fromRadians(rotation.getY()).getDegrees(),
					Rotation2d.fromRadians(rotation.getZ()).getDegrees()};
			case ROTATIONS ->
				new double[] {
					pose3d.getX(),
					pose3d.getY(),
					pose3d.getZ(),
					Rotation2d.fromRadians(rotation.getX()).getRotations(),
					Rotation2d.fromRadians(rotation.getY()).getRotations(),
					Rotation2d.fromRadians(rotation.getZ()).getRotations()};
		};
	}

	public static TimedValue<Rotation2d> visionDataToHeadingData(VisionData visionData) {
		return new TimedValue<>(visionData.getEstimatedPose().getRotation().toRotation2d(), visionData.getTimestamp());
	}

}
