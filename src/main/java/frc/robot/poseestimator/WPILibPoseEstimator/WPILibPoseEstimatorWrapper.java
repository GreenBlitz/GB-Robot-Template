package frc.robot.poseestimator.WPILibPoseEstimator;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.estimator.PoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.Odometry;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.robot.subsystems.swerve.SwerveConstants;
import frc.robot.vision.RobotPoseObservation;
import frc.robot.poseestimator.IPoseEstimator;
import frc.robot.poseestimator.OdometryData;
import frc.utils.buffers.RingBuffer.RingBuffer;
import frc.utils.math.StatisticsMath;
import org.littletonrobotics.junction.Logger;

import java.util.Optional;

public class WPILibPoseEstimatorWrapper implements IPoseEstimator {

	private final String logPath;
	private final SwerveDriveKinematics kinematics;
	private final Odometry<SwerveModulePosition[]> odometryEstimator;
	private final PoseEstimator<SwerveModulePosition[]> poseEstimator;
	private final RingBuffer<Rotation2d> poseToIMUYawDifferenceBuffer;
	private final TimeInterpolatableBuffer<Rotation2d> imuYawBuffer;
	private final TimeInterpolatableBuffer<Double> imuAccelerationBuffer;
	private RobotPoseObservation lastVisionObservation;
	private OdometryData lastOdometryData;
	private boolean isIMUOffsetCalibrated;

	public WPILibPoseEstimatorWrapper(
		String logPath,
		SwerveDriveKinematics kinematics,
		SwerveModulePosition[] initialModulePositions,
		Rotation2d initialIMUYaw,
		double initialIMUAccelerationMagnitudeG,
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
		this.lastOdometryData = new OdometryData(
			initialTimestampSeconds,
			initialModulePositions,
			Optional.of(initialIMUYaw),
			Optional.of(initialIMUAccelerationMagnitudeG)
		);
		this.isIMUOffsetCalibrated = false;
		this.poseToIMUYawDifferenceBuffer = new RingBuffer<>(WPILibPoseEstimatorConstants.POSE_TO_IMU_YAW_DIFFERENCE_BUFFER_SIZE);
		this.imuYawBuffer = TimeInterpolatableBuffer.createBuffer(WPILibPoseEstimatorConstants.IMU_YAW_BUFFER_SIZE_SECONDS);
		this.imuAccelerationBuffer = TimeInterpolatableBuffer
			.createDoubleBuffer(WPILibPoseEstimatorConstants.IMU_ACCELERATION_BUFFER_SIZE_SECONDS);
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
		lastOdometryData.setIMUAcceleration(data.getImuAccelerationMagnitudeG());

		data.getImuAccelerationMagnitudeG()
			.ifPresent((acceleration) -> imuAccelerationBuffer.addSample(lastOdometryData.getTimestampSeconds(), acceleration));
	}

	@Override
	public void updateVision(RobotPoseObservation... visionRobotPoseObservations) {
		for (RobotPoseObservation visionRobotPoseObservation : visionRobotPoseObservations) {
			updateVision(visionRobotPoseObservation);
		}
	}

	@Override
	public void resetPose(
		double timestampSeconds,
		Rotation2d imuYaw,
		double imuAccelerationMagnitudeG,
		SwerveModulePosition[] wheelPositions,
		Pose2d poseMeters
	) {
		Logger.recordOutput(logPath + "/lastPoseResetTo", poseMeters);
		poseEstimator.resetPosition(imuYaw, wheelPositions, poseMeters);
		this.lastOdometryData = new OdometryData(timestampSeconds, wheelPositions, Optional.of(imuYaw), Optional.of(imuAccelerationMagnitudeG));
		poseToIMUYawDifferenceBuffer.clear();
		imuYawBuffer.addSample(timestampSeconds, imuYaw);
		imuAccelerationBuffer.addSample(timestampSeconds, imuAccelerationMagnitudeG);
	}

	@Override
	public void resetPose(Pose2d poseMeters) {
		resetPose(
			lastOdometryData.getTimestampSeconds(),
			lastOdometryData.getIMUYaw().get(),
			lastOdometryData.getImuAccelerationMagnitudeG().get(),
			lastOdometryData.getWheelPositions(),
			poseMeters
		);
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
			getCollisionCompensatedVisionStdDevs(visionObservation)
		);
		this.lastVisionObservation = visionObservation;
	}

	private Matrix<N3, N1> getCollisionCompensatedVisionStdDevs(RobotPoseObservation visionObservation) {
		Optional<Double> imuAccelerationAtVisionObservationTimestamp = imuAccelerationBuffer.getSample(visionObservation.timestampSeconds());
		boolean isSamplePresent = imuAccelerationAtVisionObservationTimestamp.isPresent();
		boolean isColliding = imuAccelerationAtVisionObservationTimestamp.get() >= SwerveConstants.MIN_COLLISION_G_FORCE;

		return isSamplePresent && isColliding
			? WPILibPoseEstimatorConstants.DEFAULT_VISION_STD_DEV.asColumnVector()
				.minus(WPILibPoseEstimatorConstants.VISION_STD_DEV_COLLISION_REDUCTION.asColumnVector())
			: WPILibPoseEstimatorConstants.DEFAULT_VISION_STD_DEV.asColumnVector();
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
