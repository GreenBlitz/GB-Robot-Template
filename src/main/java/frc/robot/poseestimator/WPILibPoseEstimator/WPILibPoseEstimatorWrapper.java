package frc.robot.poseestimator.WPILibPoseEstimator;

import edu.wpi.first.math.estimator.PoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.Odometry;
import frc.robot.poseestimator.IPoseEstimator;
import frc.robot.poseestimator.observations.OdometryObservation;
import frc.robot.subsystems.GBSubsystem;
import frc.robot.vision.data.AprilTagVisionData;
import frc.robot.vision.data.VisionData;
import frc.utils.time.TimeUtil;
import org.littletonrobotics.junction.Logger;

import java.util.List;
import java.util.Optional;

public class WPILibPoseEstimatorWrapper extends GBSubsystem implements IPoseEstimator {

	private final SwerveDriveKinematics kinematics;
	private final Odometry<SwerveModulePosition[]> odometryEstimator;
	private final PoseEstimator<SwerveModulePosition[]> poseEstimator;
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
		this.odometryEstimator = new Odometry<>(
			kinematics,
			initialGyroAngle,
			modulePositions,
			WPILibPoseEstimatorConstants.STARTING_ODOMETRY_POSE
		);
		this.poseEstimator = new PoseEstimator<>(
			kinematics,
			odometryEstimator,
			WPILibPoseEstimatorConstants.DEFAULT_ODOMETRY_STANDARD_DEVIATIONS.asColumnVector(),
			WPILibPoseEstimatorConstants.DEFAULT_VISION_STANDARD_DEVIATIONS.asColumnVector()
		);
		this.lastOdometryObservation = new OdometryObservation(modulePositions, Optional.of(initialGyroAngle), TimeUtil.getCurrentTimeSeconds());
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
		}
		return odometryObservation.gyroAngle().get();
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
			this.lastOdometryAngle = odometryAngle;
			this.lastOdometryObservation = odometryObservation;
		}
	}

	@Override
	public void updateVision(List<AprilTagVisionData> robotPoseVisionData) {
		for (AprilTagVisionData visionData : robotPoseVisionData) {
			addVisionMeasurement(visionData);
		}
	}

	@Override
	public void resetOdometry(SwerveModulePosition[] wheelPositions, Rotation2d gyroAngle, Pose2d robotPose) {
		poseEstimator.resetPosition(gyroAngle, wheelPositions, robotPose);
		this.lastOdometryObservation = new OdometryObservation(wheelPositions, Optional.of(gyroAngle), TimeUtil.getCurrentTimeSeconds());
	}

	@Override
	public void resetPose(Pose2d newPose) {
		Logger.recordOutput(getLogPath() + "lastPoseResetTo", newPose);
		poseEstimator.resetPosition(lastOdometryAngle, lastOdometryObservation.wheelPositions(), newPose);
	}

	@Override
	public void setHeading(Rotation2d newHeading) {
		poseEstimator.resetRotation(newHeading);
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
		Logger.recordOutput(getLogPath() + "estimatedPose", getEstimatedPose());
		Logger.recordOutput(getLogPath() + "odometryPose", getOdometryPose());
		Logger.recordOutput(getLogPath() + "lastOdometryUpdate", lastOdometryObservation.timestamp());
		if (lastVisionObservation != null) {
			Logger.recordOutput(getLogPath() + "lastVisionUpdate", lastVisionObservation.getTimestamp());
		}
	}

	@Override
	protected void subsystemPeriodic() {
		log();
	}

}
