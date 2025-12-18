package frc.robot.poseestimator.WPILibPoseEstimator;

import edu.wpi.first.math.estimator.PoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.Odometry;
import frc.robot.vision.RobotPoseObservation;
import frc.robot.poseestimator.IPoseEstimator;
import frc.robot.poseestimator.OdometryData;
import frc.utils.buffers.RingBuffer.RingBuffer;
import frc.utils.math.StandardDeviations2D;
import frc.utils.math.StatisticsMath;
import org.littletonrobotics.junction.Logger;

import java.util.Optional;
import java.util.function.Function;

public class WPILibPoseEstimatorWrapper implements IPoseEstimator {

	private final String logPath;
	private final SwerveDriveKinematics kinematics;
	private final Odometry<SwerveModulePosition[]> odometryEstimator;
	private final PoseEstimator<SwerveModulePosition[]> poseEstimator;
	private final RingBuffer<Rotation2d> poseToIMUYawDifferenceBuffer;
	private final TimeInterpolatableBuffer<Rotation2d> imuYawBuffer;
	private RobotPoseObservation lastVisionObservation;
	private OdometryData lastOdometryData;
	private boolean isIMUOffsetCalibrated;

	public WPILibPoseEstimatorWrapper(
		String logPath,
		SwerveDriveKinematics kinematics,
		SwerveModulePosition[] initialModulePositions,
		Rotation2d initialIMUYaw,
		double initialTimestampSeconds
	) {
		this.logPath = logPath;
		this.kinematics = kinematics;
		this.odometryEstimator = new Odometry<>(
			kinematics,
			initialIMUYaw,
			initialModulePositions,
			WPILibPoseEstimatorConstants.STARTING_ODOMETRY_POSE
		);
		this.poseEstimator = new PoseEstimator<>(
			kinematics,
			odometryEstimator,
			WPILibPoseEstimatorConstants.DEFAULT_ODOMETRY_STD_DEV.asColumnVector(),
			WPILibPoseEstimatorConstants.DEFAULT_VISION_STD_DEV.asColumnVector()
		);
		this.lastOdometryData = new OdometryData(initialTimestampSeconds, initialModulePositions, Optional.of(initialIMUYaw));
		this.isIMUOffsetCalibrated = false;
		this.poseToIMUYawDifferenceBuffer = new RingBuffer<>(WPILibPoseEstimatorConstants.POSE_TO_IMU_YAW_DIFFERENCE_BUFFER_SIZE);
		this.imuYawBuffer = TimeInterpolatableBuffer.createBuffer(WPILibPoseEstimatorConstants.IMU_YAW_BUFFER_SIZE_SECONDS);
	}


	@Override
	public Pose2d getEstimatedPose() {
		return poseEstimator.getEstimatedPosition();
	}

	@Override
	public Optional<Pose2d> getEstimatedPoseAtTimestamp(double timestampSeconds) {
		return poseEstimator.sampleAt(timestampSeconds);
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
		data.setIMUYaw(data.getIMUYaw().orElseGet(() -> lastOdometryData.getIMUYaw().get().plus(Rotation2d.fromRadians(changeInPose.dtheta))));
		poseEstimator.updateWithTime(data.getTimestampSeconds(), data.getIMUYaw().get(), data.getWheelPositions());

		imuYawBuffer.addSample(data.getTimestampSeconds(), data.getIMUYaw().get());

		lastOdometryData.setWheelPositions(data.getWheelPositions());
		lastOdometryData.setIMUYaw(data.getIMUYaw());
		lastOdometryData.setTimestamp(data.getTimestampSeconds());
	}

	@Override
	public void updateVision(RobotPoseObservation... visionRobotPoseObservations) {
		for (RobotPoseObservation visionRobotPoseObservation : visionRobotPoseObservations) {
			updateVision(visionRobotPoseObservation);
		}
	}

	@Override
	public void resetPose(double timestampSeconds, Rotation2d imuYaw, SwerveModulePosition[] wheelPositions, Pose2d poseMeters) {
		Logger.recordOutput(logPath + "/lastPoseResetTo", poseMeters);
		poseEstimator.resetPosition(imuYaw, wheelPositions, poseMeters);
		this.lastOdometryData = new OdometryData(timestampSeconds, wheelPositions, Optional.of(imuYaw));
		poseToIMUYawDifferenceBuffer.clear();
		imuYawBuffer.addSample(timestampSeconds, imuYaw);
	}

	@Override
	public void resetPose(Pose2d poseMeters) {
		resetPose(lastOdometryData.getTimestampSeconds(), lastOdometryData.getIMUYaw().get(), lastOdometryData.getWheelPositions(), poseMeters);
	}

	@Override
	public void setHeading(Rotation2d newHeading) {
		poseEstimator.resetRotation(newHeading);
		poseToIMUYawDifferenceBuffer.clear();
	}

	@Override
	public boolean isIMUOffsetCalibrated() {
		return isIMUOffsetCalibrated;
	}

	@Override
	public void log() {
		Logger.recordOutput(logPath + "/estimatedPose", getEstimatedPose());
		Logger.recordOutput(logPath + "/odometryPose", getOdometryPose());
		Logger.recordOutput(logPath + "/lastOdometryUpdate", lastOdometryData.getTimestampSeconds());
		if (lastVisionObservation != null) {
			Logger.recordOutput(logPath + "/lastVisionUpdate", lastVisionObservation.timestampSeconds());
		}
		Logger.recordOutput(logPath + "/isIMUOffsetCalibrated", isIMUOffsetCalibrated);
	}

	public double getCurrentPoseReliability() {}

	public static double calculatePoseReliability(
		RobotPoseObservation lastVisionObservation,
		Function<Double, Pose2d> getEstimatedPoseAtTmestampFunc,
		double currentTimestamp
	) {
		Pose2d estimatedPose = getEstimatedPoseAtTmestampFunc.apply(lastVisionObservation.timestampSeconds());
		Pose2d visionEstimatedPose = lastVisionObservation.robotPose();
		StandardDeviations2D visionStdDevs = lastVisionObservation.stdDevs();

		double translationReliability = Math.max(
			estimatedPose.getTranslation().minus(visionEstimatedPose.getTranslation()).getNorm(),
			new Translation2d(visionStdDevs.xStandardDeviations(), visionStdDevs.yStandardDeviations()).getNorm()
		);
		double rotationReliability = Math
			.max(estimatedPose.getRotation().minus(visionEstimatedPose.getRotation()).getRadians(), visionStdDevs.angleStandardDeviations());
		return Math.max(translationReliability, rotationReliability);
	}

	public void resetIsIMUOffsetCalibrated() {
		poseToIMUYawDifferenceBuffer.clear();
		isIMUOffsetCalibrated = false;
	}

	private void updateVision(RobotPoseObservation visionRobotPoseObservation) {
		addVisionMeasurement(visionRobotPoseObservation);

		getEstimatedPoseToIMUYawDifference(
			imuYawBuffer.getSample(visionRobotPoseObservation.timestampSeconds()),
			visionRobotPoseObservation.timestampSeconds()
		).ifPresent(yawDifference -> {
			poseToIMUYawDifferenceBuffer.insert(yawDifference);

			if (!isIMUOffsetCalibrated) {
				updateIsIMUOffsetCalibrated();
			}
		});
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
		double poseToIMUYawDifferenceStdDev = StatisticsMath.calculateStandardDeviations(poseToIMUYawDifferenceBuffer, Rotation2d::getRadians);
		isIMUOffsetCalibrated = poseToIMUYawDifferenceStdDev < WPILibPoseEstimatorConstants.MAX_POSE_TO_IMU_YAW_DIFFERENCE_STD_DEV
			&& poseToIMUYawDifferenceBuffer.isFull();
		Logger.recordOutput(logPath + "/poseToIMUOffsetStdDev", poseToIMUYawDifferenceStdDev);
	}

	private Optional<Rotation2d> getEstimatedPoseToIMUYawDifference(Optional<Rotation2d> gyroYaw, double timestampSeconds) {
		return getEstimatedPoseAtTimestamp(timestampSeconds)
			.flatMap(estimatedPose -> gyroYaw.map(yaw -> estimatedPose.getRotation().minus(yaw)));
	}

}
