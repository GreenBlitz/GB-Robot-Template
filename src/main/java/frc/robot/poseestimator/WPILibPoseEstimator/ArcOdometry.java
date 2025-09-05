package frc.robot.poseestimator.WPILibPoseEstimator;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.Odometry;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

public class ArcOdometry extends Odometry<SwerveModulePosition[]> {

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

	public static Pose2d calculateFieldRelativeWheelPose(
		Rotation2d deltaTheta,
		SwerveModulePosition wheelPositionDelta,
		Pose2d previousWheelPose
	) {
		double circleRadiusMeters = wheelPositionDelta.distanceMeters / deltaTheta.getRadians();
		double chordLengthMeters = 2 * circleRadiusMeters * Math.sin(deltaTheta.getRadians() / 2);

		Rotation2d angleOfShortestPath = previousWheelPose.getRotation().plus(deltaTheta.div(2));

		Translation2d currentTranslation = new Translation2d(
			previousWheelPose.getX() + Math.cos(angleOfShortestPath.getRadians()) * chordLengthMeters,
			previousWheelPose.getY() + Math.sin(angleOfShortestPath.getRadians()) * chordLengthMeters
		);
		Rotation2d currentWheelHeading = previousWheelPose.getRotation().plus(deltaTheta);
		return new Pose2d(currentTranslation, currentWheelHeading);
	}

	public static Translation2d calculateRobotTranslation(
		Pose2d[] fieldRelativeWheelPoses,
		Translation2d[] robotRelativeWheelTranslations,
		Rotation2d robotOrientation
	) {
		Translation2d robotTranslation = new Translation2d();
		for (int i = 0; i < fieldRelativeWheelPoses.length; i++) {
			robotTranslation.plus(calculateRobotCenterByWheel(fieldRelativeWheelPoses[i], robotRelativeWheelTranslations[i], robotOrientation));
		}
		robotTranslation.div(fieldRelativeWheelPoses.length);
		return robotTranslation;
	}

	public static Translation2d calculateRobotCenterByWheel(
		Pose2d fieldRelativeWheelPose,
		Translation2d robotRelativeWheelTranslation,
		Rotation2d robotOrientation
	) {
		Translation2d unrotatedRobotCenter = fieldRelativeWheelPose.getTranslation().minus(robotRelativeWheelTranslation);
		return unrotatedRobotCenter.rotateAround(fieldRelativeWheelPose.getTranslation(), robotOrientation);
	}

	public static Rotation2d[] calculateWheelsDeltaThetas(
		SwerveModulePosition[] wheelPositionsDeltas,
		Rotation2d currentRobotOrientation,
		Rotation2d previousRobotOrientation
	) {
		Rotation2d[] wheelsDeltaThetas = new Rotation2d[wheelPositionsDeltas.length];
		for (int i = 0; i < wheelPositionsDeltas.length; i++) {
			wheelsDeltaThetas[i] = wheelPositionsDeltas[i].angle.plus(currentRobotOrientation.minus(previousRobotOrientation));
		}
		return wheelsDeltaThetas;
	}

	public static SwerveModulePosition[] calculateWheelPositionDeltas(
		SwerveModulePosition[] currentPositions,
		SwerveModulePosition[] previousPositions
	) {
		for (int i = 0; i < currentPositions.length; i++) {
			currentPositions[i] = new SwerveModulePosition(
				currentPositions[i].distanceMeters - previousPositions[i].distanceMeters,
				currentPositions[i].angle.minus(previousPositions[i].angle)
			);
		}
		return currentPositions;
	}

	@Override
	public Pose2d update(Rotation2d gyroAngle, SwerveModulePosition[] wheelPositions) {
		SwerveModulePosition[] wheelPositionDeltas = calculateWheelPositionDeltas(wheelPositions, previousWheelPositions);
		Rotation2d currentAngle = gyroAngle.plus(gyroOffset);
		Rotation2d[] wheelsDeltaThetas = calculateWheelsDeltaThetas(wheelPositionDeltas, currentAngle, previousRobotOrientation);

		Pose2d[] currentWheelPoses = new Pose2d[wheelPositionDeltas.length];
		for (int i = 0; i < wheelPositionDeltas.length; i++) {
			currentWheelPoses[i] = calculateFieldRelativeWheelPose(
				wheelsDeltaThetas[i],
				wheelPositionDeltas[i],
				previousFieldRelativeWheelPoses[i]
			);
		}
		Translation2d currentTranslation = calculateRobotTranslation(currentWheelPoses, kinematics.getModules(), currentAngle);

		currentRobotPoseMeters = new Pose2d(currentTranslation, currentAngle);
		kinematics.copyInto(wheelPositions, previousWheelPositions);
		previousRobotOrientation = currentAngle;
		previousFieldRelativeWheelPoses = currentWheelPoses;

		return currentRobotPoseMeters;
	}

}
