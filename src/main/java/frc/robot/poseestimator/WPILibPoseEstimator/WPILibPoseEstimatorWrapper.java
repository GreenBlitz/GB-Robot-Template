package frc.robot.poseestimator.WPILibPoseEstimator;

import edu.wpi.first.math.estimator.PoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.Odometry;
import frc.robot.vision.RobotPoseObservation;
import frc.robot.poseestimator.IPoseEstimator;
import frc.robot.poseestimator.OdometryData;
import frc.robot.subsystems.GBSubsystem;
import frc.utils.time.TimeUtil;
import org.littletonrobotics.junction.Logger;

import java.util.Optional;

public class WPILibPoseEstimatorWrapper extends GBSubsystem implements IPoseEstimator {

	private final SwerveDriveKinematics kinematics;
	private final Odometry<SwerveModulePosition[]> odometryEstimator;
	private final PoseEstimator<SwerveModulePosition[]> poseEstimator;
	private RobotPoseObservation lastVisionObservation;
	private OdometryData lastOdometryData;
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
		this.lastOdometryData = new OdometryData(modulePositions, Optional.of(initialGyroAngle), TimeUtil.getCurrentTimeSeconds());
	}


	@Override
	public Pose2d getEstimatedPose() {
		return poseEstimator.getEstimatedPosition();
	}

	@Override
	public Pose2d getEstimatedPoseAtTimestamp(double timestamp) {
		return poseEstimator.sampleAt(timestamp).orElseGet(this::getEstimatedPose);
	}

	public Rotation2d getOdometryAngle(OdometryData odometryData, Twist2d changeInPose) {
		if (odometryData.getGyroYaw().isEmpty()) {
			return lastOdometryAngle.plus(Rotation2d.fromRadians(changeInPose.dtheta));
		}
		return odometryData.getGyroYaw().get();
	}

	@Override
	public Pose2d getOdometryPose() {
		return odometryEstimator.getPoseMeters();
	}

	@Override
	public void updateOdometry(OdometryData[] odometryData) {
		for (OdometryData data : odometryData) {
			updateOdometry(data);
		}
	}

	@Override
	public void updateOdometry(OdometryData data) {
		Twist2d changeInPose = kinematics.toTwist2d(lastOdometryData.getWheelPositions(), data.getWheelPositions());
		Rotation2d odometryAngle = getOdometryAngle(data, changeInPose);
		poseEstimator.updateWithTime(data.getTimestamp(), odometryAngle, data.getWheelPositions());

		lastOdometryAngle = odometryAngle;
		lastOdometryData.setWheelPositions(data.getWheelPositions());
		lastOdometryData.setGyroYaw(data.getGyroYaw());
		lastOdometryData.setTimestamp(data.getTimestamp());
	}

	@Override
	public void updateVision(RobotPoseObservation... visionRobotPoseObservations) {
		for (RobotPoseObservation visionRobotPoseObservation : visionRobotPoseObservations) {
			addVisionMeasurement(visionRobotPoseObservation);
		}
	}

	@Override
	public void resetOdometry(SwerveModulePosition[] wheelPositions, Rotation2d gyroAngle, Pose2d robotPose) {
		poseEstimator.resetPosition(gyroAngle, wheelPositions, robotPose);
		this.lastOdometryData = new OdometryData(wheelPositions, Optional.of(gyroAngle), TimeUtil.getCurrentTimeSeconds());
	}

	@Override
	public void resetPose(Pose2d newPose) {
		Logger.recordOutput(getLogPath() + "lastPoseResetTo", newPose);
		poseEstimator.resetPosition(lastOdometryAngle, lastOdometryData.getWheelPositions(), newPose);
	}

	@Override
	public void setHeading(Rotation2d newHeading) {
		poseEstimator.resetRotation(newHeading);
	}

	private void addVisionMeasurement(RobotPoseObservation visionObservation) {
		poseEstimator.addVisionMeasurement(
			visionObservation.robotPose(),
			visionObservation.timestampSeconds(),
			visionObservation.stdDevs().asColumnVector()
		);
		this.lastVisionObservation = visionObservation;
	}

	private void log() {
		Logger.recordOutput(getLogPath() + "estimatedPose", getEstimatedPose());
		Logger.recordOutput(getLogPath() + "odometryPose", getOdometryPose());
		Logger.recordOutput(getLogPath() + "lastOdometryUpdate", lastOdometryData.getTimestamp());
		if (lastVisionObservation != null) {
			Logger.recordOutput(getLogPath() + "lastVisionUpdate", lastVisionObservation.timestampSeconds());
		}
	}

	@Override
	protected void subsystemPeriodic() {
		log();
	}

}
