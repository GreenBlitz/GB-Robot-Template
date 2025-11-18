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
import frc.utils.buffers.RingBuffer.RingBuffer;
import frc.utils.math.StatisticsMath;
import frc.utils.time.TimeUtil;
import org.littletonrobotics.junction.Logger;

import java.util.Optional;

public class WPILibPoseEstimatorWrapper implements IPoseEstimator {

	private final String logPath;
	private final SwerveDriveKinematics kinematics;
	private final Odometry<SwerveModulePosition[]> odometryEstimator;
	private final PoseEstimator<SwerveModulePosition[]> poseEstimator;
	private final RingBuffer<Rotation2d> poseToIMUAngleDifferenceBuffer;
	private RobotPoseObservation lastVisionObservation;
	private OdometryData lastOdometryData;
	private Rotation2d lastOdometryAngle;
	private boolean isIMUOffsetCalibrated;

	public WPILibPoseEstimatorWrapper(
		String logPath,
		SwerveDriveKinematics kinematics,
		SwerveModulePosition[] modulePositions,
		Rotation2d initialIMUAngle
	) {
		this.logPath = logPath;
		this.kinematics = kinematics;
		this.lastOdometryAngle = initialIMUAngle;
		this.odometryEstimator = new Odometry<>(
			kinematics,
			initialIMUAngle,
			modulePositions,
			WPILibPoseEstimatorConstants.STARTING_ODOMETRY_POSE
		);
		this.poseEstimator = new PoseEstimator<>(
			kinematics,
			odometryEstimator,
			WPILibPoseEstimatorConstants.DEFAULT_ODOMETRY_STANDARD_DEVIATIONS.asColumnVector(),
			WPILibPoseEstimatorConstants.DEFAULT_VISION_STANDARD_DEVIATIONS.asColumnVector()
		);
		this.lastOdometryData = new OdometryData(modulePositions, Optional.of(initialIMUAngle), TimeUtil.getCurrentTimeSeconds());
		this.isIMUOffsetCalibrated = false;
		this.poseToIMUAngleDifferenceBuffer = new RingBuffer<>(WPILibPoseEstimatorConstants.POSE_TO_IMU_ANGLE_DIFFERENCE_BUFFER_SIZE);
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
		Rotation2d odometryAngle = getOdometryAngle(data.getGyroYaw(), Rotation2d.fromRadians(changeInPose.dtheta));
		poseEstimator.updateWithTime(data.getTimestamp(), odometryAngle, data.getWheelPositions());

		getPoseToIMUAngleDifference(data.getGyroYaw(), data.getTimestamp()).ifPresent(poseToIMUAngleDifferenceBuffer::insert);
		updateIsIMUOffsetCalibrated();

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
		poseToIMUAngleDifferenceBuffer.clear();
	}

	@Override
	public void resetPose(Pose2d newPose) {
		Logger.recordOutput(logPath + "/lastPoseResetTo", newPose);
		poseEstimator.resetPosition(lastOdometryAngle, lastOdometryData.getWheelPositions(), newPose);
		poseToIMUAngleDifferenceBuffer.clear();
	}

	@Override
	public void setHeading(Rotation2d newHeading) {
		poseEstimator.resetRotation(newHeading);
		poseToIMUAngleDifferenceBuffer.clear();
	}

	public boolean isIMUOffsetCalibrated() {
		return isIMUOffsetCalibrated;
	}

	public Rotation2d getOdometryAngle(Optional<Rotation2d> gyroYaw, Rotation2d changeInOrientation) {
		return gyroYaw.orElseGet(() -> lastOdometryAngle.plus(changeInOrientation));
	}

	public void resetIsIMUOffsetCalibrated() {
		poseToIMUAngleDifferenceBuffer.clear();
		isIMUOffsetCalibrated = false;
	}

	private void addVisionMeasurement(RobotPoseObservation visionObservation) {
		poseEstimator.addVisionMeasurement(
			visionObservation.robotPose(),
			visionObservation.timestampSeconds(),
			visionObservation.stdDevs().asColumnVector()
		);
		this.lastVisionObservation = visionObservation;
	}

	private void updateIsIMUOffsetCalibrated() {
		double poseToIMUAngleDifferenceStdDev = StatisticsMath
			.calculateStandardDeviations(poseToIMUAngleDifferenceBuffer, Rotation2d::getRadians);
		isIMUOffsetCalibrated = poseToIMUAngleDifferenceStdDev < WPILibPoseEstimatorConstants.MAX_POSE_TO_IMU_ANGLE_DIFFERENCE_STD_DEV
			&& poseToIMUAngleDifferenceBuffer.isFull();
		Logger.recordOutput(logPath + "/poseToIMUOffsetStdDev", poseToIMUAngleDifferenceStdDev);
		Logger.recordOutput(logPath + "/isIMUOffsetCalibrated", isIMUOffsetCalibrated);
	}

	private Optional<Rotation2d> getPoseToIMUAngleDifference(Optional<Rotation2d> gyroYaw, double timeStamp) {
		return gyroYaw.map(gyroYawRotation -> getEstimatedPoseAtTimestamp(timeStamp).getRotation().minus(gyroYawRotation));
	}

	private void log() {
		Logger.recordOutput(logPath + "/estimatedPose", getEstimatedPose());
		Logger.recordOutput(logPath + "/odometryPose", getOdometryPose());
		Logger.recordOutput(logPath + "/lastOdometryUpdate", lastOdometryData.getTimestamp());
		if (lastVisionObservation != null) {
			Logger.recordOutput(logPath + "/lastVisionUpdate", lastVisionObservation.timestampSeconds());
		}
	}

}
