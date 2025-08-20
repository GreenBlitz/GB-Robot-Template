package frc.robot.poseestimator.helpers.robotheadingestimator;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import frc.robot.vision.RobotPoseObservation;
import frc.utils.math.StatisticsMath;
import frc.utils.math.PoseEstimationMath;
import frc.utils.buffers.RingBuffer.RingBuffer;
import frc.utils.TimedValue;
import frc.utils.math.AngleMath;
import org.littletonrobotics.junction.Logger;

import java.util.Optional;

public class RobotHeadingEstimator {

	private final String logPath;
	private final TimeInterpolatableBuffer<Rotation2d> unOffsetedGyroAngleInterpolator;
	private final double gyroStandardDeviation;
	private final RingBuffer<Pair<Rotation2d, Rotation2d>> estimationAndGyroBuffer;
	private Rotation2d lastGyroAngle;
	private Rotation2d estimatedHeading;
	private boolean hasFirstVisionUpdateArrived;

	public RobotHeadingEstimator(String logPath, Rotation2d initialGyroAngle, Rotation2d initialHeading, double gyroStandardDeviation) {
		this.logPath = logPath;
		this.unOffsetedGyroAngleInterpolator = TimeInterpolatableBuffer.createBuffer(RobotHeadingEstimatorConstants.POSE_BUFFER_SIZE_SECONDS);
		this.gyroStandardDeviation = gyroStandardDeviation;
		this.estimationAndGyroBuffer = new RingBuffer<>(RobotHeadingEstimatorConstants.ESTIMATION_GYRO_PAIR_BUFFER_SIZE);
		this.lastGyroAngle = initialGyroAngle;
		this.estimatedHeading = initialHeading;
		this.hasFirstVisionUpdateArrived = false;
	}

	public void reset(Rotation2d newHeading) {
		estimatedHeading = newHeading;
		unOffsetedGyroAngleInterpolator.clear();
		estimationAndGyroBuffer.clear();
	}

	public void startRecalibratingGyroOffset() {
		estimationAndGyroBuffer.clear();
	}

	public void updateByGyroReset(Rotation2d newGyroHeading) {
		Rotation2d differenceInGyroAngle = AngleMath.getAngleDifferenceWrapped(newGyroHeading, lastGyroAngle);
		lastGyroAngle = newGyroHeading;
		reset(estimatedHeading.plus(differenceInGyroAngle));
	}

	public Rotation2d getEstimatedHeading() {
		return estimatedHeading;
	}

	public Optional<Rotation2d> getEstimatedHeadingAtTimestamp(double timestamp) {
		return unOffsetedGyroAngleInterpolator.getSample(timestamp)
			.map(gyroAngleAtTimestamp -> gyroAngleAtTimestamp.plus(estimatedHeading).minus(lastGyroAngle));
	}

	public boolean isGyroOffsetCalibrated(double maximumStandardDeviationTolerance) {
		double calculatedVisionNoiseStandardDeviation = StatisticsMath.calculateStandardDeviations(
			estimationAndGyroBuffer,
			estimationVisionPair -> Math
				.abs(AngleMath.getAngleDifferenceWrapped(estimationVisionPair.getFirst(), estimationVisionPair.getSecond()).getRadians())
		);
		boolean isGyroOffsetCalibrated = calculatedVisionNoiseStandardDeviation < maximumStandardDeviationTolerance
			&& estimationAndGyroBuffer.isFull();
		Logger.recordOutput(
			logPath + RobotHeadingEstimatorConstants.VISION_NOISE_STANDARD_DEVIATION_LOGPATH_ADDITION,
			calculatedVisionNoiseStandardDeviation
		);
		Logger.recordOutput(logPath + "isGyroOffsetCalibrated/", isGyroOffsetCalibrated);
		return isGyroOffsetCalibrated;
	}

	public void updateVisionIfGyroOffsetIsNotCalibrated(
		Optional<RobotPoseObservation> visionPoseObservation,
		double visionStandardDeviation,
		double maximumStandardDeviationTolerance
	) {
		if (visionPoseObservation.isPresent() && !isGyroOffsetCalibrated(maximumStandardDeviationTolerance)) {
			updateVisionHeading(visionPoseObservation.get(), visionStandardDeviation);
			estimationAndGyroBuffer.insert(Pair.of(estimatedHeading, lastGyroAngle));
		}
	}


	public void updateVisionHeading(RobotPoseObservation visionPoseObservation, double visionStandardDeviation) {
		if (!hasFirstVisionUpdateArrived) {
			hasFirstVisionUpdateArrived = true;
			estimatedHeading = visionPoseObservation.robotPose().getRotation();
		}
		Logger.recordOutput(
			logPath + RobotHeadingEstimatorConstants.VISION_HEADING_INPUT_LOGPATH_ADDITION,
			visionPoseObservation.robotPose().getRotation()
		);
		Optional<Rotation2d> gyroAtTimestamp = unOffsetedGyroAngleInterpolator.getSample(visionPoseObservation.timestampSeconds());
		gyroAtTimestamp.ifPresent(
			gyroSampleAtTimestamp -> estimatedHeading = PoseEstimationMath.combineVisionHeadingAndGyro(
				visionPoseObservation.robotPose().getRotation(),
				gyroSampleAtTimestamp,
				lastGyroAngle,
				estimatedHeading,
				gyroStandardDeviation,
				visionStandardDeviation
			)
		);
	}

	public void updateGyroAngle(TimedValue<Rotation2d> gyroHeadingData) {
		unOffsetedGyroAngleInterpolator.addSample(gyroHeadingData.getTimestamp(), gyroHeadingData.getValue());
		estimatedHeading = estimatedHeading.plus(AngleMath.getAngleDifferenceWrapped(gyroHeadingData.getValue(), lastGyroAngle));
		lastGyroAngle = gyroHeadingData.getValue();
	}

	public void updateGyroAndVision(
		Optional<TimedValue<Rotation2d>> gyroHeadingData,
		Optional<RobotPoseObservation> visionPoseObservation,
		double visionStandardDeviation
	) {
		gyroHeadingData.ifPresent(this::updateGyroAngle);
		visionPoseObservation.ifPresent(visionObservation -> updateVisionHeading(visionObservation, visionStandardDeviation));
	}

	public void log() {
		Logger.recordOutput(logPath + RobotHeadingEstimatorConstants.ESTIMATED_HEADING_LOGPATH_ADDITION, estimatedHeading);
		Logger.recordOutput(
			logPath + RobotHeadingEstimatorConstants.ESTIMATED_HEADING_DIFFERENCE_FROM_GYRO_YAW_LOGPATH_ADDITION,
			AngleMath.getAngleDifferenceWrapped(estimatedHeading, lastGyroAngle)
		);
	}

}
