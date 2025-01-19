package frc.robot.poseestimator.helpers;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import frc.robot.constants.RobotHeadingEstimatorConstants;
import frc.robot.poseestimator.PoseEstimatorMath;
import frc.robot.poseestimator.helpers.RingBuffer.RingBuffer;
import frc.robot.vision.data.HeadingData;
import frc.utils.AngleUtils;

import java.util.Optional;

public class RobotHeadingEstimator {

	private final TimeInterpolatableBuffer<Rotation2d> unOffsetedGyroAngleInterpolator;
	private final double gyroStandardDeviation;
	private final AngleBuffer angleBuffer;
	private final RingBuffer<Pair<Rotation2d, Rotation2d>> estimationVisionBuffer;
	private Rotation2d lastGyroAngle;
	private Optional<Rotation2d> lastVisionAngle;
	private Rotation2d estimatedHeading;
	private boolean hasFirstVisionUpdateArrived;

	public RobotHeadingEstimator(Rotation2d initialGyroAngle, Rotation2d initialHeading, double gyroStandardDeviation) {
		this.unOffsetedGyroAngleInterpolator = TimeInterpolatableBuffer.createBuffer(RobotHeadingEstimatorConstants.POSE_BUFFER_SIZE_SECONDS);
		this.gyroStandardDeviation = gyroStandardDeviation;
		this.angleBuffer = new AngleBuffer(RobotHeadingEstimatorConstants.ANGLE_ACCUMULATOR_SIZE);
		this.estimationVisionBuffer = new RingBuffer<>(RobotHeadingEstimatorConstants.ESTIMATION_VISION_PAIR_BUFFER_SIZE);
		this.lastGyroAngle = initialGyroAngle;
		this.lastVisionAngle = Optional.empty();
		this.estimatedHeading = initialHeading;
		this.hasFirstVisionUpdateArrived = false;
	}

	public void reset(Rotation2d newHeading) {
		lastVisionAngle = Optional.empty();
		estimatedHeading = newHeading;
		unOffsetedGyroAngleInterpolator.clear();
	}

	public Rotation2d getEstimatedHeading() {
		return estimatedHeading;
	}

	public void updateVisionIfNotCalibrated(
		HeadingData visionHeadingData,
		double visionStandardDeviation,
		double visionComparedToEstimationStandardDeviation
	) {
		if (StandardDeviations.calculateStandardDeviations(estimationVisionBuffer, estimationVisionPair -> {
			return AngleUtils.wrappingAbs(estimationVisionPair.getFirst()).getRotations()
				- AngleUtils.wrappingAbs(estimationVisionPair.getSecond()).getRotations();
		}) < visionComparedToEstimationStandardDeviation) {
			updateVisionHeading(visionHeadingData, visionStandardDeviation);
		} else {
			lastVisionAngle = Optional.of(visionHeadingData.heading());
		}
	}


	public void updateVisionHeading(HeadingData visionHeadingData, double visionStandardDeviation) {
		angleBuffer.addAngle(visionHeadingData.heading());

		Optional<Rotation2d> slidingWindowAngleAccumulatorAverage = angleBuffer.average();
		if (
			slidingWindowAngleAccumulatorAverage.isPresent()
				&& Math.abs(
					PoseEstimatorMath.getAngleDistance(slidingWindowAngleAccumulatorAverage.get(), visionHeadingData.heading()).getRadians()
				) < RobotHeadingEstimatorConstants.VISION_HEADING_AVERAGE_COMPARISON_THRESHOLD.getRadians()
		) {
			return;
		}

		if (!hasFirstVisionUpdateArrived) {
			hasFirstVisionUpdateArrived = true;
			estimatedHeading = visionHeadingData.heading();
		}
		Optional<Rotation2d> gyroAtTimestamp = unOffsetedGyroAngleInterpolator.getSample(visionHeadingData.timestamp());
		gyroAtTimestamp.ifPresent(
			gyroSampleAtTimestamp -> estimatedHeading = PoseEstimatorMath.combineVisionHeadingToGyro(
				visionHeadingData.heading(),
				PoseEstimatorMath.getAngleDistance(gyroSampleAtTimestamp, lastGyroAngle),
				estimatedHeading,
				gyroStandardDeviation,
				visionStandardDeviation
			)
		);
		lastVisionAngle = Optional.of(visionHeadingData.heading());
	}

	public void updateGyroAngle(HeadingData gyroHeadingData) {
		unOffsetedGyroAngleInterpolator.addSample(gyroHeadingData.timestamp(), gyroHeadingData.heading());
		estimatedHeading = estimatedHeading.plus(PoseEstimatorMath.getAngleDistance(gyroHeadingData.heading(), lastGyroAngle));
		lastGyroAngle = gyroHeadingData.heading();
	}

	public void updateGyroAndVision(
		Optional<HeadingData> gyroHeadingData,
		Optional<HeadingData> visionHeadingData,
		double visionStandardDeviation
	) {
		gyroHeadingData.ifPresent(this::updateGyroAngle);
		visionHeadingData.ifPresent(visionData -> updateVisionHeading(visionData, visionStandardDeviation));
	}

	public void periodic() {
		lastVisionAngle.ifPresent(rotation2d -> estimationVisionBuffer.insert(Pair.of(estimatedHeading, rotation2d)));
	}

}
