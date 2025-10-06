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
	private final Odometry<SwerveModulePosition[]> odometryEstimatorThread;
	private final Odometry<SwerveModulePosition[]> odometryEstimator;
	private final PoseEstimator<SwerveModulePosition[]> poseEstimator;
	private RobotPoseObservation lastVisionObservation;
	private OdometryData lastOdometryDataThread;
	private Rotation2d lastOdometryAngleThread;
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
		this.lastOdometryAngleThread = initialGyroAngle;
		this.lastOdometryAngle = initialGyroAngle;
		
		this.odometryEstimatorThread = new Odometry<>(
			kinematics,
			initialGyroAngle,
			modulePositions,
			WPILibPoseEstimatorConstants.STARTING_ODOMETRY_POSE
		);

		this.odometryEstimator = new Odometry<>(
			kinematics,
			initialGyroAngle,
			modulePositions,
			WPILibPoseEstimatorConstants.STARTING_ODOMETRY_POSE
		);

		this.poseEstimator = new PoseEstimator<>(
			kinematics,
			odometryEstimatorThread,
			WPILibPoseEstimatorConstants.DEFAULT_ODOMETRY_STANDARD_DEVIATIONS.asColumnVector(),
			WPILibPoseEstimatorConstants.DEFAULT_VISION_STANDARD_DEVIATIONS.asColumnVector()
		);
		this.lastOdometryDataThread = new OdometryData(modulePositions, Optional.of(initialGyroAngle), TimeUtil.getCurrentTimeSeconds());
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
			return lastOdometryAngleThread.plus(Rotation2d.fromRadians(changeInPose.dtheta));
		}
		return odometryData.getGyroYaw().get();
	}

	@Override
	public Pose2d getOdometryPoseThread() {
		return odometryEstimatorThread.getPoseMeters();
	}
	
	public Pose2d getOdometryPose() {
		return odometryEstimator.getPoseMeters();
	}

	@Override
	public void updateAllOdometry(OdometryData[] odometryDataThread, OdometryData odometryData) {
		for (OdometryData data : odometryDataThread) {
			updateOdometryThread(data);
		}
		updateOdometry(odometryData);
	}

	@Override
	public void updateOdometryThread(OdometryData dataThread) {
		Twist2d changeInPoseThread = kinematics.toTwist2d(lastOdometryDataThread.getWheelPositions(), dataThread.getWheelPositions());
		Rotation2d odometryAngleThread = getOdometryAngle(dataThread, changeInPoseThread);
		poseEstimator.updateWithTime(dataThread.getTimestamp(), odometryAngleThread, dataThread.getWheelPositions());

		lastOdometryAngleThread = odometryAngleThread;
		lastOdometryDataThread.setWheelPositions(dataThread.getWheelPositions());
		lastOdometryDataThread.setGyroYaw(dataThread.getGyroYaw());
		lastOdometryDataThread.setTimestamp(dataThread.getTimestamp());
	}
	
	public void updateOdometry(OdometryData data) {
		Twist2d changeInPose = kinematics.toTwist2d(lastOdometryDataThread.getWheelPositions(), data.getWheelPositions());
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
		this.lastOdometryDataThread = new OdometryData(wheelPositions, Optional.of(gyroAngle), TimeUtil.getCurrentTimeSeconds());
	}

	@Override
	public void resetPose(Pose2d newPose) {
		Logger.recordOutput(getLogPath() + "lastPoseResetTo", newPose);
		poseEstimator.resetPosition(lastOdometryAngleThread, lastOdometryDataThread.getWheelPositions(), newPose);
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
		Logger.recordOutput(getLogPath() + "odometryPoseThread", getOdometryPoseThread());
		Logger.recordOutput(getLogPath() + "lastOdometryUpdate", lastOdometryDataThread.getTimestamp());
		if (lastVisionObservation != null) {
			Logger.recordOutput(getLogPath() + "lastVisionUpdate", lastVisionObservation.timestampSeconds());
		}
	}

	@Override
	protected void subsystemPeriodic() {
		log();
	}

}
