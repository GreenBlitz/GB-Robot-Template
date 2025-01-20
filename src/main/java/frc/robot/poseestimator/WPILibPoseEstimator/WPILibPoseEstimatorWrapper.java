package frc.robot.poseestimator.WPILibPoseEstimator;

import edu.wpi.first.math.estimator.PoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.*;
import frc.robot.poseestimator.IPoseEstimator;
import frc.utils.pose.PoseUtils;
import frc.robot.poseestimator.observations.OdometryObservation;
import frc.robot.subsystems.GBSubsystem;
import frc.robot.vision.data.AprilTagVisionData;
import frc.robot.vision.data.VisionData;
import frc.utils.time.TimeUtils;
import org.littletonrobotics.junction.Logger;

import java.util.List;
import java.util.Optional;

public class WPILibPoseEstimatorWrapper extends GBSubsystem implements IPoseEstimator {

	private final SwerveDriveKinematics kinematics;
	private final PoseEstimator<SwerveModulePosition[]> poseEstimator;
	private final Odometry<SwerveModulePosition[]> odometryEstimator;
	private double odometryAcceleration;
	private VisionData lastVisionObservation;
	private OdometryObservation lastOdometryObservation;
	private Rotation2d lastOdometryAngle;

	public WPILibPoseEstimatorWrapper(
		String logPath,
		SwerveDriveKinematics kinematics,
		SwerveModulePosition[] modulePositions,
		Rotation2d initialGyroAngle
	) {
		super(logPath);
		this.kinematics = kinematics;
		this.lastOdometryAngle = initialGyroAngle;
		this.poseEstimator = new PoseEstimator<>(
			kinematics,
			new Odometry<>(kinematics, initialGyroAngle, modulePositions, WPILibPoseEstimatorConstants.STARTING_ODOMETRY_POSE),
			WPILibPoseEstimatorConstants.DEFAULT_ODOMETRY_STANDARD_DEVIATIONS.asColumnVector(),
			WPILibPoseEstimatorConstants.DEFAULT_VISION_STANDARD_DEVIATIONS.asColumnVector()
		);
		this.odometryEstimator = new Odometry<>(
			kinematics,
			initialGyroAngle,
			modulePositions,
			WPILibPoseEstimatorConstants.STARTING_ODOMETRY_POSE
		);
		this.odometryAcceleration = 0;
		this.lastOdometryObservation = new OdometryObservation(
			modulePositions,
			Optional.of(initialGyroAngle),
			TimeUtils.getCurrentTimeSeconds()
		);
	}


	@Override
	public Pose2d getEstimatedPose() {
		return poseEstimator.getEstimatedPosition();
	}

	@Override
	public Pose2d getEstimatedPoseAtTimestamp(double timestamp) {
		return poseEstimator.sampleAt(timestamp).orElseGet(this::getEstimatedPose);
	}

	public Rotation2d getOdometryAngle(OdometryObservation odometryObservation, Twist2d changeInPose) {
		if (odometryObservation.gyroAngle().isEmpty()) {
			return lastOdometryAngle.plus(Rotation2d.fromRadians(changeInPose.dtheta));
		} else {
			return odometryObservation.gyroAngle().get();
		}
	}

	@Override
	public Pose2d getOdometryPose() {
		return odometryEstimator.getPoseMeters();
	}

	@Override
	public void updateOdometry(OdometryObservation[] odometryObservations) {
		for (OdometryObservation odometryObservation : odometryObservations) {
			Twist2d changeInPose = kinematics.toTwist2d(lastOdometryObservation.wheelPositions(), odometryObservation.wheelPositions());
			Rotation2d odometryAngle = getOdometryAngle(odometryObservation, changeInPose);
			poseEstimator.updateWithTime(odometryObservation.timestamp(), odometryAngle, odometryObservation.wheelPositions());
			updateOdometryPose(odometryObservation, changeInPose);

			double deltaTime = odometryObservation.timestamp() - lastOdometryObservation.timestamp();
			this.lastOdometryAngle = odometryAngle;
			this.odometryAcceleration = PoseUtils.deriveProjectedTwist(changeInPose, deltaTime);
			this.lastOdometryObservation = odometryObservation;
		}
	}

	@Override
	public void updateVision(List<AprilTagVisionData> robotPoseVisionData) {
		for (AprilTagVisionData visionData : robotPoseVisionData) {
			addVisionMeasurement(visionData);
		}
	}

	private void updateOdometryPose(OdometryObservation observation, Twist2d changeInPose) {
		odometryEstimator.update(getOdometryAngle(observation, changeInPose), observation.wheelPositions());
	}

	@Override
	public void resetOdometry(SwerveModulePosition[] wheelPositions, Rotation2d gyroAngle, Pose2d robotPose) {
		poseEstimator.resetPosition(gyroAngle, wheelPositions, robotPose);
		odometryEstimator.resetPosition(gyroAngle, wheelPositions, robotPose);
		this.lastOdometryObservation = new OdometryObservation(wheelPositions, Optional.of(gyroAngle), TimeUtils.getCurrentTimeSeconds());
	}

	@Override
	public void resetPose(Pose2d newPose) {
		Logger.recordOutput(getLogPath() + "lastPoseResetTo/", newPose);
		poseEstimator.resetPosition(lastOdometryAngle, lastOdometryObservation.wheelPositions(), newPose);
	}

	@Override
	public void setHeading(Rotation2d newHeading) {
		poseEstimator.resetRotation(newHeading);
		odometryEstimator.resetRotation(newHeading);
	}

	private void addVisionMeasurement(AprilTagVisionData visionObservation) {
		poseEstimator.addVisionMeasurement(
			visionObservation.getEstimatedPose().toPose2d(),
			visionObservation.getTimestamp(),
			WPILibPoseEstimatorConstants.VISION_STANDARD_DEVIATIONS_TRANSFORM.apply(visionObservation).asColumnVector()
		);
		this.lastVisionObservation = visionObservation;
	}

	private void log() {
		Logger.recordOutput(getLogPath() + "estimatedPose/", getEstimatedPose());
		Logger.recordOutput(getLogPath() + "odometryPose/", getOdometryPose());
		Logger.recordOutput(getLogPath() + "odometryAcceleration/", odometryAcceleration);
		Logger.recordOutput(getLogPath() + "lastOdometryUpdate/", lastOdometryObservation.timestamp());
		if (lastVisionObservation != null) {
			Logger.recordOutput(getLogPath() + "lastVisionUpdate/", lastVisionObservation.getTimestamp());
		}
	}

	@Override
	protected void subsystemPeriodic() {
		log();
	}

}
