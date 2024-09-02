package frc.robot.poseestimator;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveWheelPositions;
import frc.robot.poseestimator.observations.OdometryObservation;
import frc.robot.poseestimator.observations.VisionObservation;
import java.util.NoSuchElementException;
import java.util.Optional;

public class PoseEstimator implements IPoseEstimator {

	private final TimeInterpolatableBuffer<Pose2d> odometryPoseInterpolator;
	private final SwerveDriveKinematics kinematics;
	private final double[] odometryStandardDeviations;
	private Pose2d odometryPose;
	private Pose2d estimatedPose;
	private SwerveDriveWheelPositions lastWheelPositions;
	private Rotation2d lastGyroAngle;
	private VisionObservation lastVisionObservation;

	public PoseEstimator(
		SwerveDriveKinematics kinematics,
		SwerveDriveWheelPositions initialWheelPositions,
		Rotation2d initialGyroAngle
	) {
		this.odometryPose = new Pose2d();
		this.estimatedPose = new Pose2d();
		this.odometryPoseInterpolator = TimeInterpolatableBuffer.createBuffer(PoseEstimatorConstants.POSE_BUFFER_SIZE_SECONDS);
		this.kinematics = kinematics;
		this.lastWheelPositions = initialWheelPositions;
		this.lastGyroAngle = initialGyroAngle;
		this.odometryStandardDeviations = new double[3];
		setOdometryStandardDeviations(PoseEstimatorConstants.ODOMETRY_STANDARD_DEVIATIONS);
	}

	private void addOdometryObservation(OdometryObservation observation) {
		Twist2d twist = kinematics.toTwist2d(lastWheelPositions, observation.wheelPositions());
		twist = PoseEstimatorMath.addGyroToTwist(twist, observation.gyroAngle(), lastGyroAngle);
		this.lastGyroAngle = observation.gyroAngle();
		this.lastWheelPositions = observation.wheelPositions();
		this.odometryPose = odometryPose.exp(twist);
		this.estimatedPose = estimatedPose.exp(twist);
		odometryPoseInterpolator.addSample(observation.timestamp(), odometryPose);
	}

	private void addVisionObservation(VisionObservation observation) {
		Optional<Pose2d> odometryInterpolatedPoseSample = odometryPoseInterpolator.getSample(observation.timestamp());
		odometryInterpolatedPoseSample.ifPresent(
			pose2d -> estimatedPose = PoseEstimatorMath
				.combineVisionToOdometry(pose2d, observation, estimatedPose, odometryPose, odometryStandardDeviations)
		);
	}

	private boolean isObservationTooOld(VisionObservation visionObservation) {
		try {
			return odometryPoseInterpolator.getInternalBuffer().lastKey() - PoseEstimatorConstants.POSE_BUFFER_SIZE_SECONDS
				> visionObservation.timestamp();
		} catch (NoSuchElementException ignored) {
			return true;
		}
	}

	@Override
	public void resetPose(Pose2d initialPose) {
		this.estimatedPose = initialPose;
		this.lastGyroAngle = initialPose.getRotation();
		this.odometryPose = initialPose;
		odometryPoseInterpolator.clear();
	}

	@Override
	public Pose2d getEstimatedPose() {
		return estimatedPose;
	}

	@Override
	public void setOdometryStandardDeviations(double[] newStandardDeviations) {
		for (int row = 0; row < newStandardDeviations.length; row++) {
			odometryStandardDeviations[row] = Math.pow(newStandardDeviations[row], PoseEstimatorConstants.KALMAN_EXPONENT);
		}
	}

	@Override
	public void updateOdometry(OdometryObservation odometryObservation) {
		addOdometryObservation(odometryObservation);
	}

	@Override
	public void resetOdometry(SwerveDriveWheelPositions wheelPositions, Rotation2d gyroAngle, Pose2d pose) {
		this.lastWheelPositions = wheelPositions;
		this.lastGyroAngle = gyroAngle;
		this.odometryPose = pose;
		odometryPoseInterpolator.clear();
	}

	@Override
	public Pose2d getOdometryPose() {
		return odometryPose;
	}

	@Override
	public void updateVision(VisionObservation visionObservation) {
		this.lastVisionObservation = visionObservation;
		if (!isObservationTooOld(visionObservation)) {
			addVisionObservation(visionObservation);
		}
	}

	@Override
	public Optional<Pose2d> getVisionPose() {
		return Optional.of(lastVisionObservation.visionPose());
	}

}
