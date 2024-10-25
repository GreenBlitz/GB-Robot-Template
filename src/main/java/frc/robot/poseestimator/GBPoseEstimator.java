package frc.robot.poseestimator;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.math.kinematics.SwerveDriveWheelPositions;
import frc.robot.poseestimator.helpers.ObservationCountHelper;
import frc.robot.subsystems.GBSubsystem;
import frc.robot.vision.GyroAngleValues;
import frc.robot.vision.IVisionFilterer;
import frc.robot.poseestimator.observations.OdometryObservation;
import frc.robot.poseestimator.observations.VisionObservation;
import frc.utils.DriverStationUtils;
import frc.utils.time.TimeUtils;
import org.littletonrobotics.junction.Logger;
import java.util.List;
import java.util.NoSuchElementException;
import java.util.Objects;
import java.util.Optional;

public class GBPoseEstimator extends GBSubsystem implements IPoseEstimator {

	private final TimeInterpolatableBuffer<Pose2d> odometryPoseInterpolator;
	private final TimeInterpolatableBuffer<Pose2d> estimatedPoseInterpolator;
	private final ObservationCountHelper<Rotation2d> headingCountHelper;
	private final ObservationCountHelper<VisionObservation> poseCountHelper;
	private final IVisionFilterer visionFilterer;
	private final double[] odometryStandardDeviations;
	private OdometryValues lastOdometryValues;
	private Pose2d odometryPose;
	private Pose2d estimatedPose;
	private Rotation2d headingOffset;
	private boolean hasHeadingOffsetBeenInitialized;
	private boolean hasEstimatedPoseBeenInitialized;
	private boolean isRobotDisabled;

	public GBPoseEstimator(String logPath, IVisionFilterer visionFilterer, OdometryValues odometryValues, double[] odometryStandardDeviations) {
		super(logPath);
		this.odometryPoseInterpolator = TimeInterpolatableBuffer.createBuffer(PoseEstimatorConstants.POSE_BUFFER_SIZE_SECONDS);
		this.estimatedPoseInterpolator = TimeInterpolatableBuffer.createBuffer(PoseEstimatorConstants.POSE_BUFFER_SIZE_SECONDS);
		this.headingCountHelper = new ObservationCountHelper<>(
			visionFilterer::getAllRobotHeadingEstimations,
			PoseEstimatorConstants.VISION_OBSERVATION_COUNT_FOR_AVERAGED_POSE_CALCULATION
		);
		this.poseCountHelper = new ObservationCountHelper<>(
			visionFilterer::getAllAvailableVisionObservations,
			PoseEstimatorConstants.VISION_OBSERVATION_COUNT_FOR_AVERAGED_POSE_CALCULATION
		);
		this.visionFilterer = visionFilterer;
		this.lastOdometryValues = odometryValues;
		this.odometryStandardDeviations = new double[PoseArrayEntryValue.POSE_ARRAY_LENGTH];
		this.visionFilterer.setEstimatedPoseAtTimestampFunction(this::getEstimatedPoseAtTimeStamp);
		this.hasHeadingOffsetBeenInitialized = false;
		this.hasEstimatedPoseBeenInitialized = false;
		this.isRobotDisabled = false;
		setOdometryStandardDeviations(odometryStandardDeviations);
		calculateHeadingOffset(lastOdometryValues.gyroAngle());
		this.estimatedPose = new Pose2d();
		this.odometryPose = new Pose2d();
	}

	public IVisionFilterer getVisionFilterer() {
		return visionFilterer;
	}

	//@formatter:off
	private void calculateHeadingOffset(Rotation2d gyroAngle) {
		getEstimatedRobotHeadingByVision().ifPresentOrElse(estimatedHeading -> {
			headingOffset = estimatedHeading.minus(gyroAngle);
			hasHeadingOffsetBeenInitialized = true;
			estimatedPose = new Pose2d(estimatedPose.getTranslation(), estimatedHeading);
		}, () -> headingOffset = new Rotation2d());
	}

	private void calculateEstimatedPoseByVision() {
		getVisionPose().ifPresentOrElse(visionEstimatedPose -> {
			estimatedPose = visionEstimatedPose;
			hasEstimatedPoseBeenInitialized = true;
		}, () -> estimatedPose = new Pose2d());
	}
	//@formatter:on

	@Override
	public void resetHeadingOffset(Rotation2d newHeading) {
		if (lastOdometryValues.gyroAngle() != null) {
			headingOffset = newHeading.minus(lastOdometryValues.gyroAngle());
		}
	}

	private Optional<Rotation2d> getEstimatedRobotHeadingByVision() {
		List<Rotation2d> stackedHeadings = headingCountHelper.getStackedObservations();
		if (stackedHeadings.isEmpty()) {
			return Optional.empty();
		}
		return PoseEstimationMath.calculateAngleAverage(stackedHeadings);
	}

	@Override
	public void setOdometryStandardDeviations(double[] newStandardDeviations) {
		for (int i = 0; i < newStandardDeviations.length; i++) {
			odometryStandardDeviations[i] = newStandardDeviations[i] * newStandardDeviations[i];
		}
	}

	@Override
	public void resetOdometry(SwerveDriveWheelPositions wheelPositions, Rotation2d gyroAngle, Pose2d robotPose) {
		this.lastOdometryValues = new OdometryValues(lastOdometryValues.kinematics(), wheelPositions, gyroAngle);
		this.odometryPose = robotPose;
		odometryPoseInterpolator.clear();
	}

	@Override
	public Pose2d getOdometryPose() {
		return odometryPose;
	}

	@Override
	public Optional<Pose2d> getVisionPose() {
		List<VisionObservation> stackedObservations = poseCountHelper.getStackedObservations();
		if (stackedObservations.isEmpty()) {
			return Optional.empty();
		}
		Pose2d averagePose = PoseEstimationMath.weightedPoseMean(stackedObservations);
		Pose2d visionPose = new Pose2d(averagePose.getX(), averagePose.getY(), lastOdometryValues.gyroAngle().plus(headingOffset));
		return Optional.of(visionPose);
	}

	@Override
	public Pose2d getEstimatedPose() {
		return estimatedPose;
	}

	@Override
	public Pose2d getEstimatedPoseAtTimeStamp(double timeStamp) {
		Optional<Pose2d> estimatedPoseAtTimestamp = estimatedPoseInterpolator.getSample(timeStamp);
		return estimatedPoseAtTimestamp.orElseGet(() -> Objects.requireNonNullElseGet(estimatedPose, Pose2d::new));
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
			Pose2d currentEstimation = PoseEstimationMath.combineVisionToOdometry(
				observation,
				odometryPoseSample,
				estimatedPose,
				odometryPose,
				headingOffset,
				odometryStandardDeviations
			);
			estimatedPose = new Pose2d(currentEstimation.getTranslation(), estimatedPose.getRotation().plus(headingOffset));
			estimatedPoseInterpolator.addSample(TimeUtils.getCurrentTimeSeconds(), estimatedPose);
		});
	}

	private void addOdometryObservation(OdometryObservation observation) {
		if (!hasHeadingOffsetBeenInitialized) {
			calculateHeadingOffset(observation.gyroAngle());
		}
		updateGyroAnglesInVisionSources(observation.gyroAngle());
		Twist2d twist = lastOdometryValues.kinematics().toTwist2d(lastOdometryValues.wheelPositions(), observation.wheelsPositions());
		twist = PoseEstimationMath.addGyroToTwist(twist, observation.gyroAngle(), lastOdometryValues.gyroAngle());
		lastOdometryValues = new OdometryValues(lastOdometryValues.kinematics(), observation.wheelsPositions(), observation.gyroAngle());
		odometryPose = odometryPose.exp(twist);
		twist = PoseEstimationMath.rotateTwistToFitHeading(twist, headingOffset);
		estimatedPose = estimatedPose.exp(twist);
		odometryPoseInterpolator.addSample(observation.timestamp(), odometryPose);
	}

	private void updateGyroAnglesInVisionSources(Rotation2d gyroAngle) {
		if (gyroAngle != null) {
			Rotation2d headingWithOffset = gyroAngle.plus(headingOffset);
			visionFilterer.updateGyroAngles(new GyroAngleValues(headingWithOffset.getDegrees(), 0, 0, 0, 0, 0));
		}
	}

	public void logEstimatedPose() {
		Logger.recordOutput(super.getLogPath() + "EstimatedPose/", getEstimatedPose());
	}

	private void onEnabled() {
		hasHeadingOffsetBeenInitialized = false;
		hasEstimatedPoseBeenInitialized = false;
	}

	private void listenToEnabled() {
		if (!isRobotDisabled && DriverStationUtils.isDisabled()) {
			isRobotDisabled = true;
		} else if (isRobotDisabled && !DriverStationUtils.isDisabled()) {
			isRobotDisabled = false;
			onEnabled();
		}
	}

	@Override
	public void subsystemPeriodic() {
		listenToEnabled();
		if (!hasEstimatedPoseBeenInitialized) {
			calculateEstimatedPoseByVision();
		}
		updateVision(visionFilterer.getFilteredVisionObservations());
		logEstimatedPose();
	}

}
