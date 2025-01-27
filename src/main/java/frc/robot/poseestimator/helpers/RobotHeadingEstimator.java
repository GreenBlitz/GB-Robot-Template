package frc.robot.poseestimator.helpers;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import frc.constants.RobotHeadingEstimatorConstants;
import frc.robot.poseestimator.PoseEstimatorMath;
import frc.robot.poseestimator.helpers.RingBuffer.RingBuffer;
import frc.utils.TimedValue;

import java.util.Optional;

public class RobotHeadingEstimator {

	private final TimeInterpolatableBuffer<Rotation2d> unOffsetedGyroAngleInterpolator;
	private final double gyroStandardDeviation;
	private final RingBuffer<Pair<Rotation2d, Rotation2d>> estimationAndGyroBuffer;
	private Rotation2d lastGyroAngle;
	private Rotation2d estimatedHeading;
	private boolean hasFirstVisionUpdateArrived;

	public RobotHeadingEstimator(Rotation2d initialGyroAngle, Rotation2d initialHeading, double gyroStandardDeviation) {
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
	}

	public Rotation2d getEstimatedHeading() {
		return estimatedHeading;
	}

	public void updateVisionIfNotCalibrated(
		TimedValue<Rotation2d> visionHeadingData,
		double visionStandardDeviation,
		double maximumStandardDeviationTolerance
	) {
		if (
			StandardDeviations.calculateStandardDeviations(
				estimationAndGyroBuffer,
				estimationVisionPair -> Math
					.abs(PoseEstimatorMath.getAngleDifference(estimationVisionPair.getFirst(), estimationVisionPair.getSecond()).getRadians())
			) > maximumStandardDeviationTolerance || !estimationAndGyroBuffer.isFull()
		) {
			updateVisionHeading(visionHeadingData, visionStandardDeviation);
			estimationAndGyroBuffer.insert(Pair.of(estimatedHeading, lastGyroAngle));
		}
	}


	public void updateVisionHeading(TimedValue<Rotation2d> visionHeadingData, double visionStandardDeviation) {
		if (!hasFirstVisionUpdateArrived) {
			hasFirstVisionUpdateArrived = true;
			estimatedHeading = visionHeadingData.value();
		}
		Optional<Rotation2d> gyroAtTimestamp = unOffsetedGyroAngleInterpolator.getSample(visionHeadingData.timestamp());
		gyroAtTimestamp.ifPresent(
			gyroSampleAtTimestamp -> estimatedHeading = PoseEstimatorMath.combineVisionHeadingAndGyro(
				visionHeadingData.value(),
				gyroSampleAtTimestamp,
				lastGyroAngle,
				estimatedHeading,
				gyroStandardDeviation,
				visionStandardDeviation
			)
		);
	}

	public void updateGyroAngle(TimedValue<Rotation2d> gyroHeadingData) {
		unOffsetedGyroAngleInterpolator.addSample(gyroHeadingData.timestamp(), gyroHeadingData.value());
		estimatedHeading = estimatedHeading.plus(PoseEstimatorMath.getAngleDifference(gyroHeadingData.value(), lastGyroAngle));
		lastGyroAngle = gyroHeadingData.value();
	}

	public void updateGyroAndVision(
		Optional<TimedValue<Rotation2d>> gyroHeadingData,
		Optional<TimedValue<Rotation2d>> visionHeadingData,
		double visionStandardDeviation
	) {
		gyroHeadingData.ifPresent(this::updateGyroAngle);
		visionHeadingData.ifPresent(visionData -> updateVisionHeading(visionData, visionStandardDeviation));
	}

}
