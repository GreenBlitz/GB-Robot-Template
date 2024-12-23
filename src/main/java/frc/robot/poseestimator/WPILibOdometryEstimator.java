package frc.robot.poseestimator;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.Odometry;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import frc.robot.poseestimator.observations.OdometryObservation;
import org.littletonrobotics.junction.Logger;

public class WPILibOdometryEstimator implements IOdometryEstimator {

	private final String logPath;
	private final SwerveDriveOdometry odometryEstimator;

	public WPILibOdometryEstimator(
		String logPath,
		SwerveDriveKinematics kinematics,
		Rotation2d initialGyroAngle,
		SwerveModulePosition[] initialWheelPositions,
		Pose2d initialPose
	) {
		this.logPath = logPath;
		this.odometryEstimator = new SwerveDriveOdometry(kinematics, initialGyroAngle, initialWheelPositions, initialPose);
	}

	@Override
	public void updateOdometry(OdometryObservation[] odometryObservations) {
		for (OdometryObservation observation : odometryObservations) {
			odometryEstimator.update(observation.gyroAngle(), observation.wheelPositions());
		}
	}

	@Override
	public void resetOdometry(SwerveModulePosition[] wheelPositions, Rotation2d gyroAngle, Pose2d robotPose) {
		odometryEstimator.resetPosition(gyroAngle, wheelPositions, robotPose);
	}

	@Override
	public Pose2d getOdometryPose() {
		return odometryEstimator.getPoseMeters();
	}

	@Override
	public void setHeading(Rotation2d newHeading) {
		odometryEstimator.resetRotation(newHeading);
	}

	public void periodic() {
		Logger.recordOutput(logPath + "estimatedOdometryPose", odometryEstimator.getPoseMeters());
	}

}
