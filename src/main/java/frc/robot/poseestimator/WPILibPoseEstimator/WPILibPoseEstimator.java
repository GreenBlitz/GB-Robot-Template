package frc.robot.poseestimator.WPILibPoseEstimator;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.PoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.Odometry;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import frc.robot.poseestimator.IPoseEstimator;
import frc.robot.poseestimator.observations.OdometryObservation;
import frc.robot.subsystems.GBSubsystem;
import frc.robot.vision.data.AprilTagVisionData;

import java.util.List;
import java.util.Optional;

public class WPILibPoseEstimator extends GBSubsystem implements IPoseEstimator {

	private final PoseEstimator<SwerveModulePosition[]> poseEstimator;
	private final Odometry<SwerveModulePosition[]> odometryEstimator;

	public WPILibPoseEstimator(
		String logPath,
		SwerveDriveKinematics kinematics,
		SwerveDriveOdometry odometry,
		SwerveModulePosition[] modulePositions
	) {
		super(logPath);


		this.poseEstimator = new PoseEstimator<>(
			kinematics,
			odometry,
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
	}

	@Override
	public void resetOdometry(SwerveModulePosition[] wheelPositions, Rotation2d gyroAngle, Pose2d robotPose) {
		poseEstimator.resetPosition(gyroAngle, wheelPositions, robotPose);
		odometryEstimator.resetPosition(gyroAngle, wheelPositions, robotPose);
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
	public void updateVision(List<AprilTagVisionData> robotPoseVisionData) {}

	@Override
	public Optional<Pose2d> getVisionPose() {
		return Optional.empty();
	}

	private void updateOdometryPose(OdometryObservation observation) {
		odometryEstimator.update(observation.gyroAngle(), observation.wheelPositions());
	}

	private void addVisionMeasurement(AprilTagVisionData visionObservation) {
		poseEstimator.addVisionMeasurement(
			visionObservation.getEstimatedPose().toPose2d(),
			visionObservation.getTimestamp(),
			VecBuilder.fill(0, 0, 0) // todo change to funciton that calculats
		);
	}

}
