package frc.robot.poseestimator.WPILibPoseEstimator;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.Kinematics;
import edu.wpi.first.math.kinematics.Odometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

public class WPILibOdometryWrapper extends Odometry<SwerveModulePosition[]> {

	private final Kinematics<?, SwerveModulePosition[]> m_kinematics;
	private Pose2d m_poseMeters;

	private Rotation2d m_gyroOffset;
	private Rotation2d m_previousAngle;
	private final SwerveModulePosition[] m_previousWheelPositions;
	private Pose2d[] m_previousWheelPose2ds;

	public WPILibOdometryWrapper(
		Kinematics<?, SwerveModulePosition[]> kinematics,
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
		m_previousWheelPose2ds = wheelPose2ds;
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

	public Pose2d getWheelPose2d(Rotation2d deltaTheta, SwerveModulePosition wheelPosition, Pose2d previousWheelPose2d) {
		double circleRadiusMeters = wheelPosition.distanceMeters / deltaTheta.getRadians();
		double chordLengthMeters = 2 * circleRadiusMeters * Math.sin(deltaTheta.getRadians() / 2);

		Rotation2d angleBeta = Rotation2d.fromDegrees((180 - deltaTheta.getDegrees()) / 2);
		Rotation2d angleGama = new Rotation2d();

		Translation2d currentTranslation = new Translation2d(
			previousWheelPose2d.getX() + Math.cos(angleGama.getRadians()) * chordLengthMeters,
			previousWheelPose2d.getY() + Math.sin(angleGama.getRadians()) * chordLengthMeters
		);
		Rotation2d currentWheelHeading = previousWheelPose2d.getRotation().plus(deltaTheta);
		return new Pose2d(currentTranslation, currentWheelHeading);
	}

	public Translation2d getRobotTranslation(Pose2d[] wheelPose2ds) {
		Translation2d robotTranslation = new Translation2d();
		for (Pose2d wheelPose : wheelPose2ds) {
			robotTranslation.plus(wheelPose.getTranslation());
		}
		robotTranslation.div(wheelPose2ds.length);
		return robotTranslation;
	}

	@Override
	public Pose2d update(Rotation2d gyroAngle, SwerveModulePosition[] wheelPositions) {
		Rotation2d currentAngle = gyroAngle.plus(m_gyroOffset);
		Rotation2d deltaTheta = currentAngle.minus(m_previousAngle);

		Pose2d[] currentWheelPose2ds = new Pose2d[wheelPositions.length];
		for (int i = 0; i < wheelPositions.length; i++) {
			currentWheelPose2ds[i] = getWheelPose2d(deltaTheta, wheelPositions[i], m_previousWheelPose2ds[i]);
		}
		Translation2d currentTranslation = getRobotTranslation(currentWheelPose2ds);

		m_poseMeters = new Pose2d(currentTranslation, currentAngle);
		m_kinematics.copyInto(wheelPositions, m_previousWheelPositions);
		m_previousAngle = currentAngle;
		m_previousWheelPose2ds = currentWheelPose2ds;

		return m_poseMeters;
	}

}
