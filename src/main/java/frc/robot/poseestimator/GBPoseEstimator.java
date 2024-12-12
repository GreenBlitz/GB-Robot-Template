package frc.robot.poseestimator;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import frc.robot.poseestimator.helpers.ObservationAccumulator;
import frc.robot.poseestimator.observations.IRobotPoseVisionObservation;
import frc.robot.subsystems.GBSubsystem;
import frc.robot.poseestimator.observations.OdometryObservation;
import frc.robot.vision.multivisionsources.MultiAprilTagVisionSource;
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
	private final ObservationAccumulator<Rotation2d> headingCountHelper;
	private final ObservationAccumulator<IRobotPoseVisionObservation> poseCountHelper;
	private final MultiAprilTagVisionSource multiVisionSources;
	private final double[] odometryStandardDeviations;
	private OdometryValues lastOdometryValues;
	private Pose2d odometryPose;
	private Pose2d odometryPoseRelativeToInitialPose;
	private Pose2d estimatedPose;
	private Rotation2d headingOffset;
	private boolean hasHeadingOffsetBeenInitialized;
	private boolean hasEstimatedPoseBeenInitialized;
	private boolean isRobotDisabled;

	public GBPoseEstimator(
		String logPath,
		MultiAprilTagVisionSource multiVisionSources,
		OdometryValues odometryValues,
		double[] odometryStandardDeviations
	) {
		super(logPath);
		this.odometryPoseInterpolator = TimeInterpolatableBuffer.createBuffer(PoseEstimatorConstants.POSE_BUFFER_SIZE_SECONDS);
		this.estimatedPoseInterpolator = TimeInterpolatableBuffer.createBuffer(PoseEstimatorConstants.POSE_BUFFER_SIZE_SECONDS);
		this.multiVisionSources = multiVisionSources;
		this.headingCountHelper = new ObservationAccumulator<>(
			multiVisionSources::getRawEstimatedAngles,
			PoseEstimatorConstants.VISION_OBSERVATION_COUNT_FOR_AVERAGED_POSE_CALCULATION
		);
		this.poseCountHelper = new ObservationAccumulator<>(
			multiVisionSources::getUnfilteredVisionObservation,
			PoseEstimatorConstants.VISION_OBSERVATION_COUNT_FOR_AVERAGED_POSE_CALCULATION
		);
		this.lastOdometryValues = odometryValues;
		this.hasHeadingOffsetBeenInitialized = false;
		this.hasEstimatedPoseBeenInitialized = false;
		this.isRobotDisabled = false;
		this.odometryStandardDeviations = odometryStandardDeviations;
		calculateHeadingOffset(lastOdometryValues.gyroAngle());
		this.estimatedPose = new Pose2d();
		this.odometryPose = new Pose2d();
		this.odometryPoseRelativeToInitialPose = new Pose2d();
	}

	private void calculateHeadingOffset(Rotation2d gyroAngle) {
		getEstimatedRobotHeadingByVision().ifPresentOrElse(estimatedHeading -> {
			headingOffset = estimatedHeading.minus(gyroAngle);
			hasHeadingOffsetBeenInitialized = true;
			estimatedPose = new Pose2d(estimatedPose.getTranslation(), estimatedHeading);
		}, () -> headingOffset = new Rotation2d());
	}

	//@formatter:off
	private void calculateEstimatedPoseByVision() {
		getVisionPose().ifPresentOrElse(visionEstimatedPose -> {
			estimatedPose = visionEstimatedPose;
			hasEstimatedPoseBeenInitialized = true;
		}, () -> estimatedPose = new Pose2d());
	}
	//@formatter:on

	@Override
	public void setHeading(Rotation2d newHeading) {
		if (lastOdometryValues.gyroAngle() != null) {
			headingOffset = newHeading.minus(lastOdometryValues.gyroAngle());
		}
	}

	private Optional<Rotation2d> getEstimatedRobotHeadingByVision() {
		List<Rotation2d> stackedHeadings = headingCountHelper.getAccumulatedList();
		if (stackedHeadings.isEmpty()) {
			return Optional.empty();
		}
		return PoseEstimationMath.calculateAngleAverage(stackedHeadings);
	}

	@Override
	public void resetOdometry(SwerveModulePosition[] wheelPositions, Rotation2d gyroAngle, Pose2d robotPose) {
		this.lastOdometryValues = new OdometryValues(lastOdometryValues.kinematics(), wheelPositions, gyroAngle);
		this.odometryPose = robotPose;
		this.odometryPoseRelativeToInitialPose = robotPose;
		odometryPoseInterpolator.clear();
	}

	@Override
	public Pose2d getOdometryPose() {
		return odometryPoseRelativeToInitialPose;
	}

	@Override
	public Optional<Pose2d> getVisionPose() {
		List<IRobotPoseVisionObservation> stackedObservations = poseCountHelper.getAccumulatedList();
		if (stackedObservations.isEmpty()) {
			return Optional.empty();
		}
		Pose2d averagePose = PoseEstimationMath.poseMean(stackedObservations);
		Pose2d visionPose = new Pose2d(averagePose.getX(), averagePose.getY(), lastOdometryValues.gyroAngle().plus(headingOffset));
		return Optional.of(visionPose);
	}

	@Override
	public void resetPose(Pose2d newPose) {
		this.estimatedPose = newPose;
		this.headingOffset = newPose.getRotation().minus(lastOdometryValues.gyroAngle());
		this.odometryPoseRelativeToInitialPose = newPose;
	}

	@Override
	public Pose2d getEstimatedPose() {
		return estimatedPose;
	}

	@Override
	public Pose2d getEstimatedPoseAtTimestamp(double timeStamp) {
		Optional<Pose2d> estimatedPoseAtTimestamp = estimatedPoseInterpolator.getSample(timeStamp);
		return estimatedPoseAtTimestamp.orElseGet(() -> Objects.requireNonNullElseGet(estimatedPose, Pose2d::new));
	}

	@Override
	public void updateVision(List<IRobotPoseVisionObservation> visionObservations) {
		for (IRobotPoseVisionObservation visionObservation : visionObservations) {
			if (!isObservationTooOld(visionObservation)) {
				addVisionObservation(visionObservation);
			}
		}
	}

	@Override
	public void updateOdometry(OdometryObservation[] odometryObservations) {
		for (OdometryObservation observation : odometryObservations) {
			addOdometryObservation(observation);
		}
	}

	private boolean isObservationTooOld(IRobotPoseVisionObservation visionObservation) {
		try {
			return odometryPoseInterpolator.getInternalBuffer().lastKey() - PoseEstimatorConstants.POSE_BUFFER_SIZE_SECONDS
				> visionObservation.getTimestamp();
		} catch (NoSuchElementException ignored) {
			return true;
		}
	}

	private void addVisionObservation(IRobotPoseVisionObservation observation) {
		Optional<Pose2d> odometryInterpolatedPoseSample = odometryPoseInterpolator.getSample(observation.getTimestamp());
		odometryInterpolatedPoseSample.ifPresent(odometryPoseSample -> {
			Pose2d currentEstimation = PoseEstimationMath.combineVisionToOdometry(
				observation,
				odometryPoseSample,
				estimatedPose,
				odometryPose,
				odometryStandardDeviations,
				PoseEstimationMath.calculateStandardDeviationOfPose(observation, estimatedPose)
			);
			estimatedPose = new Pose2d(currentEstimation.getTranslation(), estimatedPose.getRotation());
			estimatedPoseInterpolator.addSample(TimeUtils.getCurrentTimeSeconds(), estimatedPose);
		});
	}

	private void addOdometryObservation(OdometryObservation observation) {
		if (!hasHeadingOffsetBeenInitialized) {
			calculateHeadingOffset(observation.gyroAngle());
		}
		multiVisionSources.updateYawInLimelights(observation.gyroAngle());
		Twist2d twist = lastOdometryValues.kinematics().toTwist2d(lastOdometryValues.wheelPositions(), observation.wheelPositions());
		twist = PoseEstimationMath.addGyroToTwist(twist, observation.gyroAngle(), lastOdometryValues.gyroAngle());
		lastOdometryValues = new OdometryValues(lastOdometryValues.kinematics(), observation.wheelPositions(), observation.gyroAngle());
		odometryPose = odometryPose.exp(twist);
		estimatedPose = estimatedPose.exp(twist);
		odometryPoseRelativeToInitialPose = odometryPoseRelativeToInitialPose.exp(twist);
		odometryPoseInterpolator.addSample(observation.timestamp(), odometryPose);
	}

	public void log() {
		Logger.recordOutput(super.getLogPath() + "EstimatedPose/", getEstimatedPose());
		Logger.recordOutput(super.getLogPath() + "OdometryPose/", getOdometryPose());
		Logger.recordOutput(super.getLogPath() + "HeadingAveragingCount/", headingCountHelper.getCount());
		Logger.recordOutput(super.getLogPath() + "PoseAveragingCount/", poseCountHelper.getCount());
		Logger.recordOutput(super.getLogPath() + "HeadingOffset/", headingOffset.getDegrees());
		Logger.recordOutput(super.getLogPath() + "hasHeadingOffsetBeenInitialized/", hasHeadingOffsetBeenInitialized);
		Logger.recordOutput(super.getLogPath() + "hasEstimatedPoseBeenInitialized/", hasEstimatedPoseBeenInitialized);
		Logger.recordOutput(super.getLogPath() + "latestGyroAngle/", lastOdometryValues.gyroAngle());
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
		updateVision(multiVisionSources.getFilteredVisionObservations());
		log();
	}

}
