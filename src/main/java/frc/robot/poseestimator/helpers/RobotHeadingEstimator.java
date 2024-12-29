package frc.robot.poseestimator.helpers;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import frc.robot.poseestimator.PoseEstimatorMath;

import java.util.Optional;

public class RobotHeadingEstimator {

	private final TimeInterpolatableBuffer<Rotation2d> unOffsetedGyroAngleInterpolator;
	private final double gyroStandardDeviation;
	private Rotation2d lastGyroAngle;
	private Rotation2d estimatedHeading;

	public RobotHeadingEstimator(Rotation2d initialGyroAngle, double gyroStandardDeviation) {
		this.lastGyroAngle = initialGyroAngle;
		this.gyroStandardDeviation = gyroStandardDeviation;
		this.unOffsetedGyroAngleInterpolator = TimeInterpolatableBuffer.createBuffer(RobotHeadingEstimatorConstants.POSE_BUFFER_SIZE_SECONDS);
		this.estimatedHeading = new Rotation2d();
	}

	public void reset(Rotation2d newHeading) {
		estimatedHeading = newHeading;
		unOffsetedGyroAngleInterpolator.clear();
	}

	public Rotation2d getEstimatedHeading() {
		return estimatedHeading;
	}

	public void updateVisionHeading(Rotation2d heading, double timestamp) {
		Optional<Rotation2d> gyroAtTimestamp = unOffsetedGyroAngleInterpolator.getSample(timestamp);
		gyroAtTimestamp.ifPresent(
			gyroSampleAtTimestamp -> estimatedHeading = PoseEstimatorMath.combineVisionHeadingToGyro(
				heading,
				PoseEstimatorMath.getAngleDistance(gyroSampleAtTimestamp, lastGyroAngle),
				estimatedHeading,
				gyroStandardDeviation,
				PoseEstimatorMath.calculateStandardDeviation(heading.getRadians(), estimatedHeading.getRadians())
			)
		);
	}

	public void updateGyroAngle(Rotation2d heading, double timestamp) {
		unOffsetedGyroAngleInterpolator.addSample(timestamp, heading);
		estimatedHeading = estimatedHeading.plus(PoseEstimatorMath.getAngleDistance(heading, lastGyroAngle));
		lastGyroAngle = heading;
	}

}
