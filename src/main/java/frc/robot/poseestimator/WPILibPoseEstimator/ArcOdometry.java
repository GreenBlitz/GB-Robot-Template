package frc.robot.poseestimator.WPILibPoseEstimator;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.Odometry;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

public class ArcOdometry extends Odometry<SwerveModulePosition[]> {

	private final SwerveDriveKinematics m_kinematics;
	private Pose2d m_poseMeters;

	private Rotation2d m_gyroOffset;
	private Rotation2d m_previousAngle;
	private final SwerveModulePosition[] m_previousWheelPositions;
	private Pose2d[] m_previousWheelFieldRelativePose2ds;

	public ArcOdometry(
		SwerveDriveKinematics kinematics,
		Rotation2d gyroAngle,
		SwerveModulePosition[] wheelPositions,
		Pose2d[] wheelPose2ds,
		Pose2d initialPoseMeters
	) {
		super(kinematics, gyroAngle, wheelPositions, initialPoseMeters);
		m_kinematics = kinematics;
		m_poseMeters = initialPoseMeters;
		m_gyroOffset = m_poseMeters.getRotation().minus(gyroAngle);
		m_previousAngle = m_poseMeters.getRotation();
		m_previousWheelPositions = m_kinematics.copy(wheelPositions);
		m_previousWheelFieldRelativePose2ds = wheelPose2ds;
	}

	@Override
	public void resetPosition(Rotation2d gyroAngle, SwerveModulePosition[] wheelPositions, Pose2d poseMeters) {
		m_poseMeters = poseMeters;
		m_previousAngle = m_poseMeters.getRotation();
		m_gyroOffset = m_poseMeters.getRotation().minus(gyroAngle);
		m_kinematics.copyInto(wheelPositions, m_previousWheelPositions);
	}

	@Override
	public void resetPose(Pose2d poseMeters) {
		m_gyroOffset = m_gyroOffset.plus(poseMeters.getRotation().minus(m_poseMeters.getRotation()));
		m_poseMeters = poseMeters;
		m_previousAngle = m_poseMeters.getRotation();
	}

	@Override
	public void resetTranslation(Translation2d translation) {
		m_poseMeters = new Pose2d(translation, m_poseMeters.getRotation());
	}

	@Override
	public void resetRotation(Rotation2d rotation) {
		m_gyroOffset = m_gyroOffset.plus(rotation.minus(m_poseMeters.getRotation()));
		m_poseMeters = new Pose2d(m_poseMeters.getTranslation(), rotation);
		m_previousAngle = m_poseMeters.getRotation();
	}

	@Override
	public Pose2d getPoseMeters() {
		return m_poseMeters;
	}

	public Pose2d getWheelPose2d(Rotation2d deltaTheta, SwerveModulePosition wheelPositionDelta, Pose2d previousWheelPose2d) {
		double circleRadiusMeters = wheelPositionDelta.distanceMeters / deltaTheta.getRadians();
		double chordLengthMeters = 2 * circleRadiusMeters * Math.sin(deltaTheta.getRadians() / 2);

		Rotation2d angleOfShortestPath = previousWheelPose2d.getRotation().plus(deltaTheta.div(2));

		Translation2d currentTranslation = new Translation2d(
			previousWheelPose2d.getX() + Math.cos(angleOfShortestPath.getRadians()) * chordLengthMeters,
			previousWheelPose2d.getY() + Math.sin(angleOfShortestPath.getRadians()) * chordLengthMeters
		);
		Rotation2d currentWheelHeading = previousWheelPose2d.getRotation().plus(deltaTheta);
		return new Pose2d(currentTranslation, currentWheelHeading);
	}

	public Translation2d getRobotTranslation(
		Pose2d[] fieldRelativeWheelPose2ds,
		Translation2d[] robotRelativeWheelTranslations,
		Rotation2d robotOrientation
	) {
		Translation2d robotTranslation = new Translation2d();
		for (int i = 0; i < fieldRelativeWheelPose2ds.length; i++) {
			robotTranslation.plus(getRobotCenterByWheel(fieldRelativeWheelPose2ds[i], robotRelativeWheelTranslations[i], robotOrientation));
		}
		robotTranslation.div(fieldRelativeWheelPose2ds.length);
		return robotTranslation;
	}

	public Translation2d getRobotCenterByWheel(
		Pose2d fieldRelativeWheelPose2d,
		Translation2d robotRelativeWheelTranslation,
		Rotation2d robotOrientation
	) {
		Translation2d unrotatedRobotCenter = fieldRelativeWheelPose2d.getTranslation().minus(robotRelativeWheelTranslation);
		return unrotatedRobotCenter.rotateAround(fieldRelativeWheelPose2d.getTranslation(), robotOrientation);
	}

	public Rotation2d[] getWheelsDeltaThetas(
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

	public SwerveModulePosition[] getWheelPositionDeltas(SwerveModulePosition[] currentPositions, SwerveModulePosition[] previousPositions) {
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
		SwerveModulePosition[] wheelPositionDeltas = getWheelPositionDeltas(wheelPositions, m_previousWheelPositions);
		Rotation2d currentAngle = gyroAngle.plus(m_gyroOffset);
		Rotation2d[] wheelsDeltaThetas = getWheelsDeltaThetas(wheelPositionDeltas, currentAngle, m_previousAngle);

		Pose2d[] currentWheelPose2ds = new Pose2d[wheelPositionDeltas.length];
		for (int i = 0; i < wheelPositionDeltas.length; i++) {
			currentWheelPose2ds[i] = getWheelPose2d(wheelsDeltaThetas[i], wheelPositionDeltas[i], m_previousWheelFieldRelativePose2ds[i]);
		}
		Translation2d currentTranslation = getRobotTranslation(currentWheelPose2ds, m_kinematics.getModules(), currentAngle);

		m_poseMeters = new Pose2d(currentTranslation, currentAngle);
		m_kinematics.copyInto(wheelPositions, m_previousWheelPositions);
		m_previousAngle = currentAngle;
		m_previousWheelFieldRelativePose2ds = currentWheelPose2ds;

		return m_poseMeters;
	}

}
