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
	private final SwerveModulePosition[] previousWheelPositions;
	private Pose2d currentRobotPoseMeters;
	private Rotation2d gyroOffset;

	public ArcOdometry(
		SwerveDriveKinematics kinematics,
		Rotation2d gyroAngle,
		SwerveModulePosition[] wheelPositions,
		Pose2d initialRobotPoseMeters
	) {
		super(kinematics, gyroAngle, wheelPositions, initialRobotPoseMeters);
		this.kinematics = kinematics;
		currentRobotPoseMeters = initialRobotPoseMeters;
		gyroOffset = currentRobotPoseMeters.getRotation().minus(gyroAngle);
		previousWheelPositions = this.kinematics.copy(wheelPositions);
	}

	@Override
	public void resetPosition(Rotation2d gyroAngle, SwerveModulePosition[] wheelPositions, Pose2d poseMeters) {
		currentRobotPoseMeters = poseMeters;
		gyroOffset = currentRobotPoseMeters.getRotation().minus(gyroAngle);
		kinematics.copyInto(wheelPositions, previousWheelPositions);
	}

	@Override
	public void resetPose(Pose2d poseMeters) {
		gyroOffset = gyroOffset.plus(poseMeters.getRotation().minus(currentRobotPoseMeters.getRotation()));
		currentRobotPoseMeters = poseMeters;
	}

	@Override
	public void resetTranslation(Translation2d translation) {
		currentRobotPoseMeters = new Pose2d(translation, currentRobotPoseMeters.getRotation());
	}

	@Override
	public void resetRotation(Rotation2d rotation) {
		gyroOffset = gyroOffset.plus(rotation.minus(currentRobotPoseMeters.getRotation()));
		currentRobotPoseMeters = new Pose2d(currentRobotPoseMeters.getTranslation(), rotation);
	}

	@Override
	public Pose2d getPoseMeters() {
		return currentRobotPoseMeters;
	}

	@Override
	public Pose2d update(Rotation2d gyroAngle, SwerveModulePosition[] wheelPositions) {
		Rotation2d currentRobotAngle = gyroAngle.plus(gyroOffset);

		Pose2d[] currentFieldRelativeWheelPoses = getCurrentFieldRelativeWheelPoses(
			currentRobotPoseMeters,
			kinematics,
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

		return currentRobotPoseMeters;
	}

	public static Pose2d[] getCurrentFieldRelativeWheelPoses(
		Pose2d previousFieldRelativeRobotPose,
		SwerveDriveKinematics kinematics,
		SwerveModulePosition[] currentWheelPositions,
		SwerveModulePosition[] previousWheelPositions
	) {
		Pose2d[] currentFieldRelativeWheelPoses = new Pose2d[currentWheelPositions.length];

		for (int i = 0; i < currentWheelPositions.length; i++) {
			currentFieldRelativeWheelPoses[i] = getCurrentFieldRelativeWheelPose(
				previousFieldRelativeRobotPose,
				kinematics.getModules()[i],
				currentWheelPositions[i],
				previousWheelPositions[i]
			);
		}
		return currentFieldRelativeWheelPoses;
	}

	public static Pose2d getCurrentFieldRelativeWheelPose(
		Pose2d previousFieldRelativeRobotPose,
		Translation2d robotCenterRelativeWheelPose,
		SwerveModulePosition currentWheelPosition,
		SwerveModulePosition previousWheelPosition
	) {
		Pose2d previousFieldRelativeWheelPose = previousFieldRelativeRobotPose
			.plus(new Transform2d(robotCenterRelativeWheelPose, new Rotation2d()))
			.rotateBy(previousWheelPosition.angle);

		Pose2d deltaWheelPose = calculateDeltaWheelPose(currentWheelPosition, previousWheelPosition);
		return previousFieldRelativeWheelPose.plus(new Transform2d(deltaWheelPose.getTranslation(), new Rotation2d()))
			.rotateBy(deltaWheelPose.getRotation());
	}

	public static Pose2d calculateDeltaWheelPose(SwerveModulePosition previousWheelPosition, SwerveModulePosition currentWheelPosition) {
		SwerveModulePosition deltaWheelPosition = getDeltaWheelPosition(previousWheelPosition, currentWheelPosition);
		Rotation2d deltaWheelOrientation = deltaWheelPosition.angle;

		double circleRadiusMeters = deltaWheelPosition.distanceMeters / deltaWheelOrientation.getRadians();
		Translation2d deltaWheelTranslation = new Translation2d(
			circleRadiusMeters * deltaWheelOrientation.getCos() - circleRadiusMeters,
			circleRadiusMeters * deltaWheelOrientation.getSin()
		);
		return new Pose2d(deltaWheelTranslation, deltaWheelOrientation);
	}

	public static SwerveModulePosition getDeltaWheelPosition(
		SwerveModulePosition previousWheelPosition,
		SwerveModulePosition currentWheelPosition
	) {
		return new SwerveModulePosition(
			currentWheelPosition.distanceMeters - previousWheelPosition.distanceMeters,
			currentWheelPosition.angle.minus(previousWheelPosition.angle)
		);
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

	public static Translation2d calculateFieldRelativeRobotCenterByWheel(
		Pose2d fieldRelativeWheelPose,
		Translation2d robotRelativeWheelTranslation,
		Rotation2d robotOrientation
	) {
		Translation2d unrotatedRobotCenter = fieldRelativeWheelPose.getTranslation().minus(robotRelativeWheelTranslation);
		return unrotatedRobotCenter.rotateAround(fieldRelativeWheelPose.getTranslation(), robotOrientation.unaryMinus());
	}

}
