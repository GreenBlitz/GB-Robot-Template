package frc.robot.poseestimator.WPILibPoseEstimator;

import edu.wpi.first.math.estimator.PoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.*;
import frc.robot.poseestimator.IPoseEstimator;
import frc.robot.poseestimator.PoseEstimationMath;
import frc.robot.poseestimator.observations.OdometryObservation;
import frc.robot.subsystems.GBSubsystem;
import frc.robot.vision.data.AprilTagVisionData;
import frc.utils.time.TimeUtils;
import org.littletonrobotics.junction.Logger;

import java.util.List;

public class WPILibPoseEstimator extends GBSubsystem implements IPoseEstimator {

	private final SwerveDriveKinematics kinematics;
	private final PoseEstimator<SwerveModulePosition[]> poseEstimator;
	private final Odometry<SwerveModulePosition[]> odometryEstimator;
	private SwerveModulePosition[] lastWheelPositions;
	private Rotation2d lastGyroAngle;
	double lastVisionUpdate;
	double lastOdometryUpdate;

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
			WPILibPoseEstimatorConstants.DEFAULT_ODOMETRY_STANDARD_DEVIATIONS.getWPILibStandardDeviations(),
			WPILibPoseEstimatorConstants.DEFAULT_VISION_STANDARD_DEVIATIONS.getWPILibStandardDeviations()
		);
		this.odometryEstimator = new Odometry<>(
			kinematics,
			initialGyroAngle,
			modulePositions,
			WPILibPoseEstimatorConstants.STARTING_ODOMETRY_POSE
		);
	}


	@Override
	public void resetPose(Pose2d newPose) {
		poseEstimator.resetPose(newPose);
	}

	@Override
	public Pose2d getEstimatedPose() {
		return poseEstimator.getEstimatedPosition();
	}

	@Override
	public Pose2d getEstimatedPoseAtTimestamp(double timestamp) {
		return poseEstimator.sampleAt(timestamp).orElseGet(this::getEstimatedPose);
	}

	public Rotation2d getOdometryAngle(OdometryObservation odometryObservation) {
		if (odometryObservation.gyroAngle().isEmpty()) {
			double changeInAngleRadians = kinematics.toTwist2d(lastWheelPositions, odometryObservation.wheelPositions()).dtheta;
			return lastGyroAngle.plus(Rotation2d.fromRadians(changeInAngleRadians));
		} else {
			return odometryObservation.gyroAngle().get();
		}
	}

	@Override
	public void updateOdometry(OdometryObservation[] odometryObservations) {
		for (OdometryObservation odometryObservation : odometryObservations) {
			Rotation2d odometryAngle = getOdometryAngle(odometryObservation);
			poseEstimator.updateWithTime(odometryObservation.timestamp(), odometryAngle, odometryObservation.wheelPositions());
			updateOdometryPose(odometryObservation);
			this.lastGyroAngle = odometryAngle;
			this.lastWheelPositions = odometryObservation.wheelPositions();
		}
		this.lastOdometryUpdate = TimeUtils.getCurrentTimeSeconds();
	}

	@Override
	public void resetOdometry(SwerveModulePosition[] wheelPositions, Rotation2d gyroAngle, Pose2d robotPose) {
		poseEstimator.resetPosition(gyroAngle, wheelPositions, robotPose);
		odometryEstimator.resetPosition(gyroAngle, wheelPositions, robotPose);
		this.lastOdometryUpdate = TimeUtils.getCurrentTimeSeconds();
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
		this.lastVisionUpdate = TimeUtils.getCurrentTimeSeconds();
	}

	private void updateOdometryPose(OdometryObservation observation) {
		odometryEstimator.update(getOdometryAngle(observation), observation.wheelPositions());
	}

	private void addVisionMeasurement(AprilTagVisionData visionObservation) {
		poseEstimator.addVisionMeasurement(
			visionObservation.getEstimatedPose().toPose2d(),
			visionObservation.getTimestamp(),
			PoseEstimationMath.calculateStandardDeviationOfPose(visionObservation, getEstimatedPose()).getWPILibStandardDeviations()
		);
	}

	private void log() {
		Logger.recordOutput(getLogPath() + "estimatedPose/", getEstimatedPose());
		Logger.recordOutput(getLogPath() + "odometryPose/", getOdometryPose());
		Logger.recordOutput(getLogPath() + "lastOdometryUpdate/", lastOdometryUpdate);
		Logger.recordOutput(getLogPath() + "lastVisionUpdate/", lastVisionUpdate);
	}

	@Override
	protected void subsystemPeriodic() {
		log();
	}

}
