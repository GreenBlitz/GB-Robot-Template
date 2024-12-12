package frc.robot.poseestimator;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.PoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.math.kinematics.Odometry;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import frc.robot.poseestimator.observations.IRobotPoseVisionObservation;
import frc.robot.poseestimator.observations.OdometryObservation;
import frc.robot.subsystems.GBSubsystem;
import frc.robot.vision.rawdata.RawAprilTagVisionData;

import java.util.List;
import java.util.Optional;

public class WPILibPoseEstimator extends GBSubsystem implements IPoseEstimator {
	
	private final PoseEstimator<SwerveModulePosition[]> poseEstimator;
	private final Odometry<SwerveModulePosition[]> odometryEstimator;
	
	private final TimeInterpolatableBuffer<Pose2d> timeInterpolatableBuffer;
	
	public WPILibPoseEstimator(String logPath, SwerveDriveKinematics kinematics, SwerveDriveOdometry odometry, SwerveModulePosition[] modulePositions) {
		super(logPath);
		
		
		this.poseEstimator = new PoseEstimator<>(
				kinematics,
				odometry,
				PoseEstimatorConstants.DEFAULT_ODOMETRY_STANDARD_DEVIATIONS
						.getWPILibStandardDeviations(),
				PoseEstimatorConstants.DEFAULT_VISION_STANDARD_DEVIATIONS
						.getWPILibStandardDeviations()
		);
		this.odometryEstimator = new Odometry<>(
				kinematics,
				PoseEstimatorConstants.STARTING_ODOMETRY_ANGLE,
				modulePositions,
				PoseEstimatorConstants.STARTING_ODOMETRY_POSE
		);
		
		this.timeInterpolatableBuffer = TimeInterpolatableBuffer.createBuffer(PoseEstimatorConstants.POSE_BUFFER_SIZE_SECONDS);;
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
		return timeInterpolatableBuffer.getSample(timestamp).orElseGet(this::getEstimatedPose);
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
	public void updateVision(List<RawAprilTagVisionData> robotPoseVisionData) {
	
	}
	
	@Override
	public Optional<Pose2d> getVisionPose() {
		return Optional.empty();
	}
	
	private void updateOdometryPose(OdometryObservation observation) {
		odometryEstimator.update(observation.gyroAngle(), observation.wheelPositions());
	}
	
	private void addVisionMeasurement(RawAprilTagVisionData visionObservation){
		poseEstimator.addVisionMeasurement(
				visionObservation.getEstimatedPose().toPose2d(),
				visionObservation.getTimestamp(),
				VecBuilder.fill(0,0,0) //todo change to funciton that calculats
		);
	}
}
