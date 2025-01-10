package frc.robot.poseestimator.WPILibPoseEstimator;

import edu.wpi.first.math.estimator.PoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.*;
import frc.robot.poseestimator.IPoseEstimator;
import frc.robot.poseestimator.PoseEstimationMath;
import frc.robot.poseestimator.helpers.StandardDeviations2D;
import frc.robot.poseestimator.observations.OdometryObservation;
import frc.robot.subsystems.GBSubsystem;
import frc.robot.vision.data.AprilTagVisionData;
import frc.robot.vision.data.VisionData;
import frc.utils.time.TimeUtils;
import org.littletonrobotics.junction.Logger;

import java.util.List;
import java.util.Optional;

public class WPILibPoseEstimator extends GBSubsystem implements IPoseEstimator {

	private final SwerveDriveKinematics kinematics;
	private final PoseEstimator<SwerveModulePosition[]> poseEstimator;
	private final Odometry<SwerveModulePosition[]> odometryEstimator;
	private double visionSpeed;
	private double odometrySpeed;
	private VisionData lastVisionObservation;
	private OdometryObservation lastOdometryObservation;
	private Rotation2d lastGyroAngle;

	public WPILibPoseEstimator(
		String logPath,
		SwerveDriveKinematics kinematics,
		SwerveModulePosition[] modulePositions,
		Rotation2d initialGyroAngle
	) {
		super(logPath);
		this.kinematics = kinematics;
		this.lastGyroAngle = initialGyroAngle;
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
		this.visionSpeed = 0;
		this.odometrySpeed = 0;
		this.lastOdometryObservation = new OdometryObservation(
			modulePositions,
			Optional.of(initialGyroAngle),
			TimeUtils.getCurrentTimeSeconds()
		);
		this.lastVisionObservation = new VisionData(new Pose3d(), TimeUtils.getCurrentTimeSeconds());
	}


	@Override
	public void resetPose(Pose2d newPose) {
		Logger.recordOutput(getLogPath() + "lastPoseResetedTo/", newPose);
		poseEstimator.resetPosition(lastGyroAngle, lastOdometryObservation.wheelPositions(), newPose);
	}

	@Override
	public Pose2d getEstimatedPose() {
		return poseEstimator.getEstimatedPosition();
//		return new Pose2d(poseEstimator.getEstimatedPosition().getTranslation(), lastGyroAngle);
	}

	@Override
	public Pose2d getEstimatedPoseAtTimestamp(double timestamp) {
		return poseEstimator.sampleAt(timestamp).orElseGet(this::getEstimatedPose);
	}

	public Rotation2d getOdometryAngle(OdometryObservation odometryObservation, Twist2d changeInPose) {
		if (odometryObservation.gyroAngle().isEmpty()) {
			return lastGyroAngle.plus(Rotation2d.fromRadians(changeInPose.dtheta));
		} else {
			return odometryObservation.gyroAngle().get();
		}
	}

	@Override
	public void updateOdometry(OdometryObservation[] odometryObservations) {
		for (OdometryObservation odometryObservation : odometryObservations) {
			Twist2d dPose = kinematics.toTwist2d(lastOdometryObservation.wheelPositions(), odometryObservation.wheelPositions());
			Rotation2d odometryAngle = getOdometryAngle(odometryObservation, dPose);
			poseEstimator.updateWithTime(odometryObservation.timestamp(), odometryAngle, odometryObservation.wheelPositions());
			updateOdometryPose(odometryObservation, dPose);
			this.lastGyroAngle = odometryAngle;
			this.odometrySpeed = PoseEstimationMath.deriveTwist(dPose, odometryObservation.timestamp() - lastOdometryObservation.timestamp());
			this.lastOdometryObservation = odometryObservation;
		}
	}

	@Override
	public void resetOdometry(SwerveModulePosition[] wheelPositions, Rotation2d gyroAngle, Pose2d robotPose) {
		poseEstimator.resetPosition(gyroAngle, wheelPositions, robotPose);
		odometryEstimator.resetPosition(gyroAngle, wheelPositions, robotPose);
		this.lastOdometryObservation = new OdometryObservation(wheelPositions, Optional.of(gyroAngle), TimeUtils.getCurrentTimeSeconds());
	}

	@Override
	public Pose2d getOdometryPose() {
		return odometryEstimator.getPoseMeters();
	}

	@Override
	public void setHeading(Rotation2d newHeading) {
		poseEstimator.resetRotation(newHeading);
		odometryEstimator.resetRotation(newHeading);
	}

	@Override
	public void updateVision(List<AprilTagVisionData> robotPoseVisionData) {
		for (AprilTagVisionData visionData : robotPoseVisionData) {
			addVisionMeasurement(visionData);
		}
	}

	private void updateOdometryPose(OdometryObservation observation, Twist2d dPose) {
		odometryEstimator.update(getOdometryAngle(observation, dPose), observation.wheelPositions());
	}

	private void addVisionMeasurement(AprilTagVisionData visionObservation) {
		poseEstimator.addVisionMeasurement(
			visionObservation.getEstimatedPose().toPose2d(),
			visionObservation.getTimestamp(),
			new StandardDeviations2D(visionObservation.getDistanceFromAprilTagMeters() * WPILibPoseEstimatorConstants.VISION_STDEVS_FACTOR).asColumnVector()
		);
		this.visionSpeed = PoseEstimationMath.deriveVisionData(lastVisionObservation, visionObservation);
		this.lastVisionObservation = visionObservation;
	}

	private void log() {
		Logger.recordOutput(getLogPath() + "estimatedPose/", getEstimatedPose());
		Logger.recordOutput(getLogPath() + "odometryPose/", getOdometryPose());
		Logger.recordOutput(getLogPath() + "visionSpeed/", visionSpeed);
		Logger.recordOutput(getLogPath() + "odometrySpeed/", odometrySpeed);
		Logger.recordOutput(getLogPath() + "speedsDifferance", odometrySpeed - visionSpeed);
		Logger.recordOutput(getLogPath() + "lastOdometryUpdate/", lastOdometryObservation.timestamp());
		Logger.recordOutput(getLogPath() + "lastVisionUpdate/", lastVisionObservation.getTimestamp());
	}

	@Override
	protected void subsystemPeriodic() {
		log();
	}

}
