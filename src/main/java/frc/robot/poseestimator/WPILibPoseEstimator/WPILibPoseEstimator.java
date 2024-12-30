package frc.robot.poseestimator.WPILibPoseEstimator;

import edu.wpi.first.math.estimator.PoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.Odometry;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import frc.robot.poseestimator.IPoseEstimator;
import frc.robot.poseestimator.PoseEstimationMath;
import frc.robot.poseestimator.observations.OdometryObservation;
import frc.robot.subsystems.GBSubsystem;
import frc.robot.vision.data.AprilTagVisionData;
import frc.utils.time.TimeUtils;
import org.littletonrobotics.junction.Logger;

import java.util.List;

public class WPILibPoseEstimator extends GBSubsystem implements IPoseEstimator {

	private final PoseEstimator<SwerveModulePosition[]> poseEstimator;
	private final Odometry<SwerveModulePosition[]> odometryEstimator;
	double lastVisionUpdate;
	double lastOdometryUpdate;

	public WPILibPoseEstimator(String logPath, SwerveDriveKinematics kinematics, SwerveModulePosition[] modulePositions) {
		super(logPath);
		this.poseEstimator = new PoseEstimator<>(
			kinematics,
			new Odometry<>(
				kinematics,
				WPILibPoseEstimatorConstants.STARTING_ODOMETRY_ANGLE,
				modulePositions,
				WPILibPoseEstimatorConstants.STARTING_ODOMETRY_POSE
			),
			WPILibPoseEstimatorConstants.DEFAULT_ODOMETRY_STANDARD_DEVIATIONS.getWPILibStandardDeviations(),
			WPILibPoseEstimatorConstants.DEFAULT_VISION_STANDARD_DEVIATIONS.getWPILibStandardDeviations()
		);
		this.odometryEstimator = new Odometry<>(
			kinematics,
			WPILibPoseEstimatorConstants.STARTING_ODOMETRY_ANGLE,
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

	@Override
	public void updateOdometry(OdometryObservation[] odometryObservations) {
		for (OdometryObservation odometryObservation : odometryObservations) {
			poseEstimator.update(odometryObservation.gyroAngle(), odometryObservation.wheelPositions());
			updateOdometryPose(odometryObservation);
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
		odometryEstimator.update(observation.gyroAngle(), observation.wheelPositions());
	}

	private void addVisionMeasurement(AprilTagVisionData visionObservation) {
		poseEstimator.addVisionMeasurement(
			visionObservation.getEstimatedPose().toPose2d(),
			visionObservation.getTimestamp(),
			PoseEstimationMath.calculateStandardDeviationOfPose(visionObservation, getEstimatedPose()).getWPILibStandardDeviations()
		);
	}

	@Override
	protected void subsystemPeriodic() {
		Logger.recordOutput(getLogPath() + "estimatedPose/", getEstimatedPose());
		Logger.recordOutput(getLogPath() + "odometryPose/", getOdometryPose());
		Logger.recordOutput(getLogPath() + "lastVisionUpdate/", lastVisionUpdate);
		Logger.recordOutput(getLogPath() + "lastOdometryUpdate/", lastOdometryUpdate);
	}

}
