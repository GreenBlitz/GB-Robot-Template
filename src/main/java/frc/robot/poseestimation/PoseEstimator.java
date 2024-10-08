package frc.robot.poseestimation;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import frc.robot.poseestimation.observations.OdometryObservation;
import frc.robot.poseestimation.posecalculation.PoseCalculator;
import org.littletonrobotics.junction.Logger;

import java.util.function.Consumer;


public class PoseEstimator {

	private final PoseCalculator poseCalculator;
	private final Consumer<Rotation2d> resetSwerveHeading;

	public PoseEstimator(Consumer<Rotation2d> resetSwerveHeading, SwerveDriveKinematics kinematics) {
		this.poseCalculator = new PoseCalculator(PoseEstimatorConstants.ODOMETRY_STANDARD_DEVIATIONS, kinematics);
		this.resetSwerveHeading = resetSwerveHeading;
		resetPose(PoseEstimatorConstants.DEFAULT_POSE);
	}

	public Pose2d getCurrentPose() {
		return poseCalculator.getEstimatedPose();
	}

	public void resetPose(Pose2d currentPose) {
		resetSwerveHeading.accept(currentPose.getRotation());
		poseCalculator.resetPose(currentPose);
	}

	public void resetHeading(Rotation2d targetAngle) {
		resetPose(new Pose2d(getCurrentPose().getTranslation(), targetAngle));
	}

	private void logCurrentPose() {
		Logger.recordOutput(PoseEstimatorConstants.LOG_PATH + "EstimatedPose", getCurrentPose());
	}

	private void logCurrentOdometryPose() {
		Logger.recordOutput(PoseEstimatorConstants.LOG_PATH + "OdometryPose", getCurrentPose());
	}


	/**
	 * Updates the pose estimator with the given swerve wheel positions and gyro rotations. This function accepts an array of swerve wheel
	 * positions and an array of gyro rotations because the odometry can be updated at a faster rate than the main loop (which is 50 hertz). This
	 * means you could have a couple of odometry updates per main loop, and you would want to update the pose estimator with all of them.
	 */
	public void updatePoseEstimatorOdometry(OdometryObservation[] odometryObservations) {
		for (OdometryObservation odometryObservation : odometryObservations) {
			poseCalculator.addOdometryObservation(odometryObservation);
		}
	}

	public void updatePoseEstimator(OdometryObservation[] odometryObservations) {
		updatePoseEstimatorOdometry(odometryObservations);

		logCurrentPose();
		logCurrentOdometryPose();
	}

}
