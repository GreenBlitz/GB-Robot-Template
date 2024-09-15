package frc.robot.poseestimator;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveWheelPositions;
import frc.robot.poseestimator.observations.OdometryObservation;
import frc.robot.poseestimator.observations.VisionObservation;
import org.littletonrobotics.junction.Logger;

import java.util.List;
import java.util.NoSuchElementException;
import java.util.Optional;

public class GBPoseEstimator implements IPoseEstimator {

	private final TimeInterpolatableBuffer<Pose2d> odometryPoseInterpolator;
	private final SwerveDriveKinematics kinematics;
	private final double[] odometryStandardDeviations;
	private Pose2d odometryPose;
	private Pose2d estimatedPose;
	private SwerveDriveWheelPositions lastWheelPositions;
	private Rotation2d lastGyroAngle;
	private VisionObservation lastVisionObservation;

	public GBPoseEstimator(
		SwerveDriveKinematics kinematics,
		SwerveDriveWheelPositions initialWheelPositions,
		Rotation2d initialGyroAngle,
		double[] odometryStandardDeviations
	) {
		this.odometryPose = new Pose2d();
		this.estimatedPose = new Pose2d();
		this.odometryPoseInterpolator = TimeInterpolatableBuffer.createBuffer(PoseEstimatorConstants.POSE_BUFFER_SIZE_SECONDS);
		this.kinematics = kinematics;
		this.lastWheelPositions = initialWheelPositions;
		this.lastGyroAngle = initialGyroAngle;
		this.odometryStandardDeviations = new double[3];
		setOdometryStandardDeviations(odometryStandardDeviations);
	}

	@Override
	public void setOdometryStandardDeviations(double[] newStandardDeviations) {
		for (int i = 0; i < newStandardDeviations.length; i++) {
			odometryStandardDeviations[i] = newStandardDeviations[i] * newStandardDeviations[i];
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
	public Optional<Pose2d> getVisionPose() {
		return Optional.of(lastVisionObservation.visionPose());
	}

	@Override
	public Pose2d getEstimatedPose() {
		return estimatedPose;
	}


	@Override
	public void updateVision(List<VisionObservation> visionObservations) {
		for (VisionObservation visionObservation : visionObservations) {
			if (!isObservationTooOld(visionObservation)) {
				addVisionObservation(visionObservation);
			}
		}
	}

	@Override
	public void updateOdometry(List<OdometryObservation> odometryObservation) {
		for (OdometryObservation observation : odometryObservation) {
			addOdometryObservation(observation);
			logEstimatedPose();
		}
	}

	@Override
	public void updatePoseEstimator(List<OdometryObservation> odometryObservation, List<VisionObservation> visionObservations) {
		updateOdometry(odometryObservation);
		updateVision(visionObservations);
	}

	private boolean isObservationTooOld(VisionObservation visionObservation) {
		try {
			return odometryPoseInterpolator.getInternalBuffer().lastKey() - PoseEstimatorConstants.POSE_BUFFER_SIZE_SECONDS
				> visionObservation.timestamp();
		} catch (NoSuchElementException ignored) {
			return true;
		}
	}

	private void addVisionObservation(VisionObservation observation) {
		this.lastVisionObservation = observation;
		Optional<Pose2d> odometryInterpolatedPoseSample = odometryPoseInterpolator.getSample(observation.timestamp());
		odometryInterpolatedPoseSample.ifPresent(
			pose2d -> estimatedPose = PoseEstimatorMath
				.combineVisionToOdometry(pose2d, observation, estimatedPose, odometryPose, odometryStandardDeviations)
		);
	}

	private void addOdometryObservation(OdometryObservation observation) {
		Twist2d twist = kinematics.toTwist2d(lastWheelPositions, observation.wheelsPositions());
		twist = PoseEstimatorMath.addGyroToTwist(twist, observation.gyroAngle(), lastGyroAngle);
		this.lastGyroAngle = observation.gyroAngle();
		this.lastWheelPositions = observation.wheelsPositions();
		this.odometryPose = odometryPose.exp(twist);
		this.estimatedPose = estimatedPose.exp(twist);
		odometryPoseInterpolator.addSample(observation.timestamp(), odometryPose);
	}

	public void logEstimatedPose() {
		Logger.recordOutput(PoseEstimatorConstants.LOG_PATH + "EstimatedPose", getEstimatedPose());
	}

}
