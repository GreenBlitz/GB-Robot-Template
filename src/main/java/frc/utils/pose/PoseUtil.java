package frc.utils.pose;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.constants.MathConstants;
import frc.robot.poseestimator.Pose2dComponentsValue;
import frc.robot.poseestimator.Pose3dComponentsValue;
import frc.utils.AngleUnit;
import frc.utils.alerts.Alert;
import frc.utils.math.ToleranceMath;

public class PoseUtil {

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

	public static boolean getIsColliding(Translation2d imuAccelerationG, double minimumCollisionIMUAccelerationG) {
		return imuAccelerationG.getNorm() >= minimumCollisionIMUAccelerationG;
	}

	public static boolean getIsTilted(Rotation2d imuRoll, Rotation2d imuPitch, Rotation2d minimumTiltIMURoll, Rotation2d minimumTiltIMUPitch) {
		return Math.abs(imuRoll.getRadians()) >= minimumTiltIMURoll.getRadians()
			|| Math.abs(imuPitch.getRadians()) >= minimumTiltIMUPitch.getRadians();
	}

	public static boolean getIsSkidding(
		SwerveDriveKinematics kinematics,
		SwerveModuleState[] moduleStates,
		double minimumSkidRobotToModuleVelocityDifferenceMetersPerSecond
	) {
		ChassisSpeeds swerveVelocity = kinematics.toChassisSpeeds(moduleStates);
		Translation2d swerveTranslationalVelocityMetersPerSecond = new Translation2d(
			swerveVelocity.vxMetersPerSecond,
			swerveVelocity.vyMetersPerSecond
		);

		SwerveModuleState[] moduleRotationalStates = kinematics
			.toSwerveModuleStates(new ChassisSpeeds(0, 0, swerveVelocity.omegaRadiansPerSecond));
		SwerveModuleState[] moduleTranslationalStates = getModuleTranslationalStates(moduleStates, moduleRotationalStates);

		for (SwerveModuleState moduleTranslationalState : moduleTranslationalStates) {
			if (
				!ToleranceMath.isNear(
					swerveTranslationalVelocityMetersPerSecond,
					new Translation2d(moduleTranslationalState.speedMetersPerSecond, moduleTranslationalState.angle),
					minimumSkidRobotToModuleVelocityDifferenceMetersPerSecond
				)
			) {
				return true;
			}
		}
		return false;
	}

	private static SwerveModuleState[] getModuleTranslationalStates(
		SwerveModuleState[] moduleStates,
		SwerveModuleState[] moduleRotationalStates
	) {
		SwerveModuleState[] moduleTranslationalStates = new SwerveModuleState[Math.min(moduleStates.length, moduleRotationalStates.length)];
		for (int i = 0; i < moduleTranslationalStates.length; i++) {
			moduleTranslationalStates[i] = getModuleTranslationalState(moduleStates[i], moduleRotationalStates[i]);
		}
		return moduleTranslationalStates;
	}

	private static SwerveModuleState getModuleTranslationalState(SwerveModuleState moduleState, SwerveModuleState moduleRotationalState) {
		Translation2d moduleTranslationalVelocity = new Translation2d(moduleState.speedMetersPerSecond, moduleState.angle)
			.minus(new Translation2d(moduleRotationalState.speedMetersPerSecond, moduleRotationalState.angle));
		return moduleTranslationalVelocity.getNorm() > MathConstants.MAXIMUM_NEGLIGIBLE_VECTOR_NORM
			? new SwerveModuleState(moduleTranslationalVelocity.getNorm(), moduleTranslationalVelocity.getAngle())
			: new SwerveModuleState();
	}

}
