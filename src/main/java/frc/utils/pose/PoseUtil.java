package frc.utils.pose;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.poseestimator.Pose2dComponentsValue;
import frc.robot.poseestimator.Pose3dComponentsValue;
import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.subsystems.swerve.SwerveMath;
import frc.utils.AngleUnit;
import frc.utils.alerts.Alert;

import frc.utils.math.ToleranceMath;
import org.littletonrobotics.junction.Logger;

public class PoseUtil {

	private static final String IS_AT_POSE_LOG_PATH_PREFIX = "isAtPoses";

	public static Pose2d EMPTY_POSE2D = new Pose2d(Double.NaN, Double.NaN, Rotation2d.fromDegrees(Double.NaN));

	public static boolean isAtPose(
		Pose2d currentPose,
		Pose2d targetPose,
		ChassisSpeeds currentSpeeds,
		Pose2d tolerances,
		Pose2d deadbands,
		String logPath
	) {
		boolean isAtX = MathUtil.isNear(targetPose.getX(), currentPose.getX(), tolerances.getX());
		boolean isAtY = MathUtil.isNear(targetPose.getY(), currentPose.getY(), tolerances.getY());
		boolean isAtHeading = ToleranceMath.isNearWrapped(targetPose.getRotation(), currentPose.getRotation(), tolerances.getRotation());
		boolean isStill = SwerveMath.isStill(currentSpeeds, deadbands);

		Logger.recordOutput(IS_AT_POSE_LOG_PATH_PREFIX + logPath + "/isAtX", isAtX);
		Logger.recordOutput(IS_AT_POSE_LOG_PATH_PREFIX + logPath + "/isAtY", isAtY);
		Logger.recordOutput(IS_AT_POSE_LOG_PATH_PREFIX + logPath + "/isAtHeading", isAtHeading);
		Logger.recordOutput(IS_AT_POSE_LOG_PATH_PREFIX + logPath + "/isStill", isStill);

		return isAtX && isAtY && isAtHeading && isStill;
	}

	public static boolean isAtTranslation(Translation2d currentPose, Translation2d targetPose, Translation2d tolerances) {
		boolean isAtX = MathUtil.isNear(targetPose.getX(), currentPose.getX(), tolerances.getX());
		boolean isAtY = MathUtil.isNear(targetPose.getY(), currentPose.getY(), tolerances.getY());
		return isAtX && isAtY;
	}

	public static Pose3d toPose3D(double[] poseArray, AngleUnit angleUnit) {
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

	public static Pose2d toPose2D(double[] poseArray, AngleUnit angleUnit) {
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
		double[] translationArray = translation3DToTranslationArray(pose3d.getTranslation());
		double[] rotationArray = rotation3DToRotationArray(pose3d.getRotation(), angleUnit);
		double[] poseArray = new double[Pose3dComponentsValue.POSE3D_COMPONENTS_AMOUNT];
		System.arraycopy(translationArray, 0, poseArray, 0, translationArray.length);
		System.arraycopy(rotationArray, 0, poseArray, translationArray.length, rotationArray.length);
		return poseArray;
	}

	public static double[] translation3DToTranslationArray(Translation3d translation3d) {
		return new double[] {translation3d.getX(), translation3d.getY(), translation3d.getZ(),};
	}

	public static double[] rotation3DToRotationArray(Rotation3d rotation3d, AngleUnit angleUnit) {
		return switch (angleUnit) {
			case RADIANS -> new double[] {rotation3d.getX(), rotation3d.getY(), rotation3d.getZ(),};
			case DEGREES ->
				new double[] {
					Rotation2d.fromRadians(rotation3d.getX()).getDegrees(),
					Rotation2d.fromRadians(rotation3d.getY()).getDegrees(),
					Rotation2d.fromRadians(rotation3d.getZ()).getDegrees()};
			case ROTATIONS ->
				new double[] {
					Rotation2d.fromRadians(rotation3d.getX()).getRotations(),
					Rotation2d.fromRadians(rotation3d.getY()).getRotations(),
					Rotation2d.fromRadians(rotation3d.getZ()).getRotations()};
		};
	}

}
