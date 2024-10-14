package frc.robot.poseestimator;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveWheelPositions;
import frc.robot.subsystems.GBSubsystem;
import frc.robot.vision.limelights.GyroAngleValues;
import frc.robot.vision.limelights.ILimelightFilterer;
import frc.robot.poseestimator.observations.OdometryObservation;
import frc.robot.poseestimator.observations.VisionObservation;
import frc.utils.time.TimeUtils;
import org.littletonrobotics.junction.Logger;
import java.util.List;
import java.util.NoSuchElementException;
import java.util.Optional;

public class GBPoseEstimator extends GBSubsystem implements IPoseEstimator {

	private final TimeInterpolatableBuffer<Pose2d> odometryPoseInterpolator;
	private final TimeInterpolatableBuffer<Pose2d> estimatedPoseInterpolator;
	private final ILimelightFilterer limelightFilterer;
	private final SwerveDriveKinematics kinematics;
	private final double[] odometryStandardDeviations;
	private Pose2d odometryPose;
	private Pose2d estimatedPose;
	private SwerveDriveWheelPositions latestWheelPositions;
	private Rotation2d latestGyroAngle;
	private Rotation2d headingOffset;
	private boolean hasHeadingOffsetBeenInitialized;

	public GBPoseEstimator(
		String logPath,
		ILimelightFilterer limelightFilterer,
		SwerveDriveKinematics kinematics,
		SwerveDriveWheelPositions initialWheelPositions,
		Rotation2d initialGyroAngle,
		double[] odometryStandardDeviations
	) {
		super(logPath);
		this.odometryPoseInterpolator = TimeInterpolatableBuffer.createBuffer(PoseEstimatorConstants.POSE_BUFFER_SIZE_SECONDS);
		this.estimatedPoseInterpolator = TimeInterpolatableBuffer.createBuffer(PoseEstimatorConstants.POSE_BUFFER_SIZE_SECONDS);
		this.limelightFilterer = limelightFilterer;
		this.kinematics = kinematics;
		this.latestWheelPositions = initialWheelPositions;
		this.latestGyroAngle = initialGyroAngle;
		this.odometryStandardDeviations = new double[PoseArrayEntryValue.POSE_ARRAY_LENGTH];
		this.limelightFilterer.setEstimatedPoseAtTimestampFunction(this::getEstimatedPoseAtTimeStamp);
		this.hasHeadingOffsetBeenInitialized = false;
		setOdometryStandardDeviations(odometryStandardDeviations);
		calculateHeadingOffset(initialGyroAngle);
		//@formatter:off
		getVisionPose().ifPresentOrElse(calculatedPose -> {
			this.odometryPose = calculatedPose;
			this.estimatedPose = calculatedPose;
		}, () -> {
			this.odometryPose = new Pose2d();
			this.estimatedPose = new Pose2d();
		});
		//@formatter:on
	}

	public ILimelightFilterer getLimelightFilterer() {
		return limelightFilterer;
	}

	//@formatter:off
	public void calculateHeadingOffset(Rotation2d gyroAngle) {
		Optional<Rotation2d> estimatedRobotHeading = getEstimatedRobotHeadingByVision();
		estimatedRobotHeading.ifPresentOrElse(estimatedHeading -> {
			headingOffset = estimatedHeading.minus(gyroAngle);
			hasHeadingOffsetBeenInitialized = true;
		}, () -> headingOffset = new Rotation2d());
	}
	//@formatter:on

	@Override
	public void resetHeadingOffset(Rotation2d newHeading) {
		if (latestGyroAngle != null) {
			headingOffset = newHeading.minus(latestGyroAngle);
		}
	}

	private Optional<Rotation2d> getEstimatedRobotHeadingByVision() {
		List<Rotation2d> stackedHeadingEstimations = limelightFilterer.getAllRobotHeadingEstimations();
		List<Rotation2d> headingEstimation = stackedHeadingEstimations;
		while (
			stackedHeadingEstimations.size() < PoseEstimatorConstants.VISION_OBSERVATION_COUNT_FOR_AVERAGED_POSE_CALCULATION
				&& !headingEstimation.isEmpty()
		) {
			if (!stackedHeadingEstimations.contains(headingEstimation.get(0))) {
				stackedHeadingEstimations.addAll(headingEstimation);
			}
			headingEstimation = limelightFilterer.getAllRobotHeadingEstimations();
		}
		return PoseEstimationMath.calculateAngleAverage(stackedHeadingEstimations);
	}

	@Override
	public void setOdometryStandardDeviations(double[] newStandardDeviations) {
		for (int i = 0; i < newStandardDeviations.length; i++) {
			odometryStandardDeviations[i] = newStandardDeviations[i] * newStandardDeviations[i];
		}
	}

	@Override
	public void resetOdometry(SwerveDriveWheelPositions wheelPositions, Rotation2d gyroAngle, Pose2d robotPose) {
		this.latestWheelPositions = wheelPositions;
		this.latestGyroAngle = gyroAngle;
		this.odometryPose = robotPose;
		odometryPoseInterpolator.clear();
	}

	@Override
	public Pose2d getOdometryPose() {
		return odometryPose;
	}

	@Override
	public Optional<Pose2d> getVisionPose() {
		List<VisionObservation> stackedRawData = limelightFilterer.getAllAvailableLimelightRawData();
		List<VisionObservation> rawData = limelightFilterer.getAllAvailableLimelightRawData();
		while (stackedRawData.size() < PoseEstimatorConstants.VISION_OBSERVATION_COUNT_FOR_AVERAGED_POSE_CALCULATION && !rawData.isEmpty()) {
			if (!stackedRawData.contains(rawData.get(0))) {
				stackedRawData.addAll(rawData);
			}
			rawData = limelightFilterer.getAllAvailableLimelightRawData();
		}
		if (stackedRawData.isEmpty()) {
			return Optional.empty();
		}
		Pose2d averagePose = PoseEstimationMath.weightedPoseMean(stackedRawData);
		Pose2d visionPose = new Pose2d(averagePose.getX(), averagePose.getY(), latestGyroAngle.plus(headingOffset));
		return Optional.of(visionPose);
	}

	@Override
	public Pose2d getEstimatedPose() {
		return estimatedPose;
	}

	@Override
	public Pose2d getEstimatedPoseAtTimeStamp(double timeStamp) {
		Optional<Pose2d> estimatedPoseAtTimestamp = estimatedPoseInterpolator.getSample(timeStamp);
		return estimatedPoseAtTimestamp.orElseGet(() -> estimatedPose);
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
	public void updateOdometry(List<OdometryObservation> odometryObservations) {
		for (OdometryObservation observation : odometryObservations) {
			addOdometryObservation(observation);
		}
		logEstimatedPose();
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
		Optional<Pose2d> odometryInterpolatedPoseSample = odometryPoseInterpolator.getSample(observation.timestamp());
		odometryInterpolatedPoseSample.ifPresent(odometryPoseSample -> {
			Pose2d currentEstimation = PoseEstimationMath
				.combineVisionToOdometry(observation, odometryPoseSample, estimatedPose, odometryPose, odometryStandardDeviations);
			estimatedPose = new Pose2d(currentEstimation.getTranslation(), odometryPoseSample.getRotation());
			estimatedPoseInterpolator.addSample(TimeUtils.getCurrentTimeSeconds(), estimatedPose);
		});
	}

	private void addOdometryObservation(OdometryObservation observation) {
		if (!hasHeadingOffsetBeenInitialized) {
			calculateHeadingOffset(observation.gyroAngle());
		}
		updateGyroAnglesInLimeLight(observation.gyroAngle());
		Twist2d twist = kinematics.toTwist2d(latestWheelPositions, observation.wheelsPositions());
		twist = PoseEstimationMath.addGyroToTwist(twist, observation.gyroAngle().plus(headingOffset), latestGyroAngle.plus(headingOffset));
		latestGyroAngle = observation.gyroAngle();
		latestWheelPositions = observation.wheelsPositions();
		odometryPose = odometryPose.exp(twist);
		estimatedPose = estimatedPose.exp(twist);
		odometryPoseInterpolator.addSample(observation.timestamp(), odometryPose);
	}

	private void updateGyroAnglesInLimeLight(Rotation2d gyroAngle) {
		if (gyroAngle != null) {
			Rotation2d headingWithOffset = gyroAngle.plus(headingOffset);
			limelightFilterer.updateGyroAngles(new GyroAngleValues(headingWithOffset.getDegrees(), 0, 0, 0, 0, 0));
		}
	}

	public void logEstimatedPose() {
		Logger.recordOutput(super.getLogPath() + "EstimatedPose/", getEstimatedPose());
	}

	@Override
	public void subsystemPeriodic() {
		updateVision(limelightFilterer.getFilteredVisionObservations());
	}

}
