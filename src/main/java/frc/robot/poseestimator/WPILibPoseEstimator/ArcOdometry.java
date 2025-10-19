package frc.robot.poseestimator.WPILibPoseEstimator;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

public class ArcOdometry extends SwerveDriveOdometry {

	private final SwerveDriveKinematics kinematics;
	private Pose2d currentRobotPoseMeters;

	private Rotation2d gyroOffset;
	private Rotation2d previousRobotOrientation;
	private final SwerveModulePosition[] previousWheelPositions;
	private Pose2d[] previousFieldRelativeWheelPoses;

	public ArcOdometry(
		SwerveDriveKinematics kinematics,
		Rotation2d gyroAngle,
		SwerveModulePosition[] wheelPositions,
		Pose2d[] fieldRelativeWheelPoses,
		Pose2d initialRobotPoseMeters
	) {
		super(kinematics, gyroAngle, wheelPositions, initialRobotPoseMeters);
		this.kinematics = kinematics;
		currentRobotPoseMeters = initialRobotPoseMeters;
		gyroOffset = currentRobotPoseMeters.getRotation().minus(gyroAngle);
		previousRobotOrientation = currentRobotPoseMeters.getRotation();
		previousWheelPositions = this.kinematics.copy(wheelPositions);
		previousFieldRelativeWheelPoses = fieldRelativeWheelPoses;
	}

	@Override
	public void resetPosition(Rotation2d gyroAngle, SwerveModulePosition[] wheelPositions, Pose2d poseMeters) {
		currentRobotPoseMeters = poseMeters;
		previousRobotOrientation = currentRobotPoseMeters.getRotation();
		gyroOffset = currentRobotPoseMeters.getRotation().minus(gyroAngle);
		kinematics.copyInto(wheelPositions, previousWheelPositions);
	}

	@Override
	public void resetPose(Pose2d poseMeters) {
		gyroOffset = gyroOffset.plus(poseMeters.getRotation().minus(currentRobotPoseMeters.getRotation()));
		currentRobotPoseMeters = poseMeters;
		previousRobotOrientation = currentRobotPoseMeters.getRotation();
	}

	@Override
	public void resetTranslation(Translation2d translation) {
		currentRobotPoseMeters = new Pose2d(translation, currentRobotPoseMeters.getRotation());
	}

	@Override
	public void resetRotation(Rotation2d rotation) {
		gyroOffset = gyroOffset.plus(rotation.minus(currentRobotPoseMeters.getRotation()));
		currentRobotPoseMeters = new Pose2d(currentRobotPoseMeters.getTranslation(), rotation);
		previousRobotOrientation = currentRobotPoseMeters.getRotation();
	}

	@Override
	public Pose2d getPoseMeters() {
		return currentRobotPoseMeters;
	}

	@Override
	public Pose2d update(Rotation2d gyroAngle, SwerveModulePosition[] wheelPositions) {
		Rotation2d currentRobotAngle = gyroAngle.plus(gyroOffset);

		Pose2d[] currentFieldRelativeWheelPoses = getNewFieldRelativeWheelPoses(
			previousFieldRelativeWheelPoses,
			wheelPositions,
			previousWheelPositions
		);

		Translation2d currentRobotTranslation = calculateFieldRelativeRobotCenterByWheels(
			currentFieldRelativeWheelPoses,
			kinematics.getModules(),
			currentRobotAngle
		);

		currentRobotPoseMeters = new Pose2d(currentRobotTranslation, currentRobotAngle);
		kinematics.copyInto(wheelPositions, previousWheelPositions);
		previousRobotOrientation = currentRobotAngle;
		previousFieldRelativeWheelPoses = currentFieldRelativeWheelPoses;

		return currentRobotPoseMeters;
	}

	public static SwerveModulePosition getDeltaWheelPosition(SwerveModulePosition previousWheelPosition, SwerveModulePosition newWheelPosition) {
		return new SwerveModulePosition(
				newWheelPosition.distanceMeters - previousWheelPosition.distanceMeters,
				newWheelPosition.angle.minus(previousWheelPosition.angle)
		);
	}

	public static Transform2d calculateDeltaWheelPose(SwerveModulePosition previousWheelPosition, SwerveModulePosition newWheelPosition) {
		SwerveModulePosition deltaWheelPosition = getDeltaWheelPosition(previousWheelPosition, newWheelPosition);
		Rotation2d deltaWheelOrientation = deltaWheelPosition.angle;

		double circleRadiusMeters = deltaWheelPosition.distanceMeters / deltaWheelOrientation.getRadians();
		Translation2d deltaWheelTranslation = new Translation2d(
				circleRadiusMeters * deltaWheelOrientation.getCos() - circleRadiusMeters,
				circleRadiusMeters * deltaWheelOrientation.getSin()
		);
		return new Transform2d(deltaWheelTranslation, deltaWheelOrientation);
	}

	public static Pose2d[] getNewFieldRelativeWheelPoses(
			Pose2d[] previousFieldRelativeWheelPoses,
			SwerveModulePosition[] newWheelPositions,
			SwerveModulePosition[] previousWheelPositions
	) {
		for (int i = 0; i < previousFieldRelativeWheelPoses.length; i++) {
			previousFieldRelativeWheelPoses[i] = previousFieldRelativeWheelPoses[i]
					.plus(calculateDeltaWheelPose(newWheelPositions[i], previousWheelPositions[i]));
		}
		return previousFieldRelativeWheelPoses;
	}

	public static Translation2d calculateFieldRelativeRobotCenterByWheel(
			Pose2d fieldRelativeWheelPose,
			Translation2d robotRelativeWheelTranslation,
			Rotation2d robotOrientation
	) {
		Translation2d unrotatedRobotCenter = fieldRelativeWheelPose.getTranslation().minus(robotRelativeWheelTranslation);
		return unrotatedRobotCenter.rotateAround(fieldRelativeWheelPose.getTranslation(), robotOrientation.unaryMinus());
	}

	public static Translation2d calculateFieldRelativeRobotCenterByWheels(
			Pose2d[] fieldRelativeWheelPoses,
			Translation2d[] robotRelativeWheelTranslations,
			Rotation2d robotOrientation
	) {
		Translation2d robotTranslation = new Translation2d();
		for (int i = 0; i < fieldRelativeWheelPoses.length; i++) {
			robotTranslation
					.plus(calculateFieldRelativeRobotCenterByWheel(fieldRelativeWheelPoses[i], robotRelativeWheelTranslations[i], robotOrientation));
		}
		robotTranslation.div(fieldRelativeWheelPoses.length);
		return robotTranslation;
	}

}
