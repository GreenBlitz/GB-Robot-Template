package frc.robot.poseestimator;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.Odometry;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import frc.robot.poseestimator.observations.OdometryObservation;
import frc.robot.subsystems.GBSubsystem;

public class WPILibOdometryEstimator extends GBSubsystem implements IOdometryEstimator {

	private final Odometry<SwerveModulePosition[]> odometryEstimator;

	public WPILibOdometryEstimator(
		String logPath,
		SwerveDriveKinematics kinematics,
		Rotation2d initialGyroAngle,
		SwerveModulePosition[] initialWheelPositions,
		Pose2d initialPose
	) {
		super(logPath);
		this.odometryEstimator = new Odometry<>(kinematics, initialGyroAngle, initialWheelPositions, initialPose);
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

}
