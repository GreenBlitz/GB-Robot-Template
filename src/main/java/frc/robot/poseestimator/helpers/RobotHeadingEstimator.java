package frc.robot.poseestimator.helpers;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import frc.robot.poseestimator.PoseEstimatorMath;
import frc.robot.poseestimator.WPILibPoseEstimator.WPILibPoseEstimatorConstants;
import org.littletonrobotics.junction.Logger;

import java.util.Optional;

public class RobotHeadingEstimator {

	private final TimeInterpolatableBuffer<Rotation2d> unOffsetedGyroAngleInterpolator;
	private final double gyroStandardDeviation;
	private Rotation2d lastGyroAngle;
	private Rotation2d lastVisionAngle;
	private Rotation2d estimatedHeading;

	public RobotHeadingEstimator(Rotation2d initialGyroAngle, double gyroStandardDeviation) {
		this.lastGyroAngle = initialGyroAngle;
		this.lastVisionAngle = initialGyroAngle;
		this.gyroStandardDeviation = gyroStandardDeviation;
		this.unOffsetedGyroAngleInterpolator = TimeInterpolatableBuffer.createBuffer(RobotHeadingEstimatorConstants.POSE_BUFFER_SIZE_SECONDS);
		this.estimatedHeading = WPILibPoseEstimatorConstants.INITIAL_GYRO_ANGLE;
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
			gyroSampleAtTimestamp -> {
/*				if(gyroSampleAtTimestamp.minus(lastGyroAngle).getDegrees() < 3 &&
					heading.getDegrees() < lastVisionAngle.getDegrees()
				)*/
				estimatedHeading = PoseEstimatorMath.combineVisionHeadingToGyro(
					heading,
					PoseEstimatorMath.getAngleDistance(gyroSampleAtTimestamp, lastGyroAngle),
					estimatedHeading,
					gyroStandardDeviation,
					0.03
//				PoseEstimatorMath.calculateStandardDeviation(heading.getRadians(), estimatedHeading.getRadians())
				);
			}
		);
	}

	public void updateGyroAngle(Rotation2d heading, double timestamp) {
		unOffsetedGyroAngleInterpolator.addSample(timestamp, heading);
		estimatedHeading = estimatedHeading.plus(PoseEstimatorMath.getAngleDistance(heading, lastGyroAngle));
		lastGyroAngle = heading;
		Logger.recordOutput("RobotHeadingEstimator/gyro", new Pose2d(new Translation2d(0, 0), heading));
		Logger.recordOutput("RobotHeadingEstimator/estimate", new Pose2d(new Translation2d(0, 0), estimatedHeading));
	}

}
