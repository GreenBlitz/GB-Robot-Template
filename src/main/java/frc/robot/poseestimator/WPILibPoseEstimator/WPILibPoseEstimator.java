package frc.robot.poseestimator.WPILibPoseEstimator;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.PoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.math.kinematics.Odometry;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import frc.robot.poseestimator.IPoseEstimator;
import frc.robot.poseestimator.PoseEstimationMath;
import frc.robot.poseestimator.PoseEstimatorConstants;
import frc.robot.poseestimator.helpers.ProcessedVisionData;
import frc.robot.poseestimator.helpers.VisionDenoiser;
import frc.robot.poseestimator.helpers.dataswitcher.VisionObservationSwitcher;
import frc.robot.poseestimator.observations.OdometryObservation;
import frc.robot.subsystems.GBSubsystem;
import frc.robot.vision.multivisionsources.MultiAprilTagVisionSource;
import frc.robot.vision.rawdata.AprilTagVisionData;
import frc.utils.time.TimeUtils;

import java.util.List;
import java.util.Optional;

public class WPILibPoseEstimator extends GBSubsystem implements IPoseEstimator {

	private final TimeInterpolatableBuffer<Pose2d> odometryPoseInterpolator;
	private final TimeInterpolatableBuffer<Pose2d> timeInterpolatableBuffer;
	private final TimeInterpolatableBuffer<Double> accelerationInterpolator;

	private final PoseEstimator<SwerveModulePosition[]> poseEstimator;
	private final Odometry<SwerveModulePosition[]> odometryEstimator;
	private final MultiAprilTagVisionSource multiVisionSources;
	private final VisionDenoiser visionDenoiser;
	private final VisionObservationSwitcher visionObservationSwitcher;

	public WPILibPoseEstimator(
		String logPath,
		SwerveDriveKinematics kinematics,
		SwerveDriveOdometry odometry,
		SwerveModulePosition[] modulePositions,
		MultiAprilTagVisionSource multiAprilTagVisionSource,
		VisionDenoiser visionDenoiser
	) {
		super(logPath);
		this.accelerationInterpolator = TimeInterpolatableBuffer.createDoubleBuffer(PoseEstimatorConstants.POSE_BUFFER_SIZE_SECONDS);
		this.odometryPoseInterpolator = TimeInterpolatableBuffer.createBuffer(PoseEstimatorConstants.POSE_BUFFER_SIZE_SECONDS);
		this.timeInterpolatableBuffer = TimeInterpolatableBuffer.createBuffer(WPILibPoseEstimatorConstants.POSE_BUFFER_SIZE_SECONDS);

		this.visionDenoiser = visionDenoiser;
		this.visionObservationSwitcher = new VisionObservationSwitcher(
			() -> visionDenoiser.calculateFixedObservationByOdometryLinearFilter(getOdometryPose(), odometryPoseInterpolator),
			visionDenoiser::calculateLinearFilterResult,
			PoseEstimatorConstants.DATA_CHANGE_RATE,
			PoseEstimatorConstants.DATA_SWITCHING_DURATION
		);
		this.multiVisionSources = multiAprilTagVisionSource;
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
		;
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
	public void updateVision(List<AprilTagVisionData> robotPoseVisionData) {
		for (AprilTagVisionData visionData : robotPoseVisionData) {
			addVisionMeasurement(visionData);
		}
	}

	@Override
	public Optional<Pose2d> getVisionPose() {
		return Optional.empty();
	}

	private void updateOdometryPose(OdometryObservation observation) {
		odometryEstimator.update(observation.gyroAngle(), observation.wheelPositions());
		odometryPoseInterpolator.addSample(observation.timestamp(), getOdometryPose());
	}

	private void addVisionMeasurement(AprilTagVisionData visionObservation) {
		visionDenoiser.addVisionObservation(visionObservation);

		if (accelerationInterpolator.getSample(visionObservation.getTimestamp()).orElse(0.0) < PoseEstimatorConstants.ACCELERATION_TOLERANCE) {
			visionObservationSwitcher.switchToFirstSource();
		} else {
			visionObservationSwitcher.switchToSecondsSource();
		}

		ProcessedVisionData fixedObservation = visionObservationSwitcher.getValue(visionObservation.getTimestamp())
			.orElse(PoseEstimationMath.processVisionData(visionObservation, getEstimatedPose()));
		poseEstimator.addVisionMeasurement(fixedObservation.getEstimatedPose(), fixedObservation.getTimestamp(), fixedObservation.getStdDev().getWPILibStandardDeviations());
	}

	@Override
	protected void subsystemPeriodic() {
		updateVision(multiVisionSources.getFilteredVisionData());
	}
}
