package frc.robot.poseestimator.helpers;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import frc.robot.constants.RobotHeadingEstimatorConstants;
import frc.robot.poseestimator.PoseEstimatorMath;
import frc.robot.vision.data.HeadingData;

import java.util.Optional;

public class RobotHeadingEstimator {

	private final TimeInterpolatableBuffer<Rotation2d> unOffsetedGyroAngleInterpolator;
	private final double gyroStandardDeviation;
	private Rotation2d lastGyroAngle;
	private Rotation2d estimatedHeading;
	private boolean hasFirstVisionUpdateArrived;

	public RobotHeadingEstimator(Rotation2d initialGyroAngle, Rotation2d initialHeading, double gyroStandardDeviation) {
		this.gyroStandardDeviation = gyroStandardDeviation;
		this.lastGyroAngle = initialGyroAngle;
		this.estimatedHeading = initialHeading;
		this.hasFirstVisionUpdateArrived = false;
		this.unOffsetedGyroAngleInterpolator = TimeInterpolatableBuffer.createBuffer(RobotHeadingEstimatorConstants.POSE_BUFFER_SIZE_SECONDS);
	}

	public void reset(Rotation2d newHeading) {
		estimatedHeading = newHeading;
		unOffsetedGyroAngleInterpolator.clear();
	}

	public Rotation2d getEstimatedHeading() {
		return estimatedHeading;
	}

	public void updateVisionHeading(HeadingData visionHeadingData, double visionStandardDeviation) {
		if(!hasFirstVisionUpdateArrived) {
			hasFirstVisionUpdateArrived = true;
			estimatedHeading = visionHeadingData.heading();
		}
		Optional<Rotation2d> gyroAtTimestamp = unOffsetedGyroAngleInterpolator.getSample(visionHeadingData.timestamp());
		gyroAtTimestamp.ifPresent(
			gyroSampleAtTimestamp -> estimatedHeading = PoseEstimatorMath.combineVisionHeadingToGyro(
				visionHeadingData.heading(),
				PoseEstimatorMath.getAngleDistance(lastGyroAngle, gyroSampleAtTimestamp),
				estimatedHeading,
				gyroStandardDeviation,
				visionStandardDeviation
			)
		);
	}

	public void updateGyroAngle(HeadingData gyroHeadingData) {
		unOffsetedGyroAngleInterpolator.addSample(gyroHeadingData.timestamp(), gyroHeadingData.heading());
		estimatedHeading = estimatedHeading.plus(PoseEstimatorMath.getAngleDistance(gyroHeadingData.heading(), lastGyroAngle));
		lastGyroAngle = gyroHeadingData.heading();
	}

}
