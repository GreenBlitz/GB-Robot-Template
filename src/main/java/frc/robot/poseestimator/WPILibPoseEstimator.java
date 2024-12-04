package frc.robot.poseestimator;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.PoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.math.kinematics.Odometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.robot.poseestimator.observations.OdometryObservation;
import frc.robot.poseestimator.observations.VisionObservation;
import frc.robot.subsystems.GBSubsystem;
import frc.robot.vision.MultiVisionSources;
import frc.robot.vision.VisionFilterer;
import frc.robot.vision.VisionFiltererConfig;
import frc.utils.time.TimeUtils;

import java.util.List;
import java.util.Optional;

public class WPILibPoseEstimator extends GBSubsystem implements IPoseEstimator {

	private final VisionFilterer visionFilterer;
	private final PoseEstimator<SwerveModulePosition[]> wpiPoseEstimator;
	private final TimeInterpolatableBuffer<Pose2d> estimatedPoseInterpolator;
	private OdometryValues lastOdometryValues;
	private Pose2d odometryPose;
	private Pose2d lastVisionPose;

	public WPILibPoseEstimator(
		String logPath,
		MultiVisionSources multiVisionSources,
		VisionFiltererConfig visionFiltererConfig,
		OdometryValues odometryValues,
		Odometry<SwerveModulePosition[]> odometry,
		Matrix<N3, N1> odometryStandardDeviations,
		Matrix<N3, N1> defaultVisionStandardDeviations
	) {
		super(logPath);
		this.visionFilterer = new VisionFilterer(
			visionFiltererConfig,
			multiVisionSources,
			this::getEstimatedPoseAtTimestamp
		);
		this.lastOdometryValues = odometryValues;
		this.wpiPoseEstimator = new PoseEstimator<>(
			odometryValues.kinematics(),
			odometry,
			odometryStandardDeviations,
			defaultVisionStandardDeviations
		);
		this.estimatedPoseInterpolator = TimeInterpolatableBuffer.createBuffer(PoseEstimatorConstants.POSE_BUFFER_SIZE_SECONDS);
		this.headingOffset = new Rotation2d();
		this.odometryPose = odometry.getPoseMeters();
	}

	@Override
	public void resetPose(Pose2d newPose) {
		wpiPoseEstimator.resetPose(newPose);
	}

	@Override
	public Pose2d getEstimatedPose() {
		return wpiPoseEstimator.getEstimatedPosition();
	}

	@Override
	public Pose2d getEstimatedPoseAtTimestamp(double timestamp) {
		Optional<Pose2d> estimatedPoseAtTimestamp = estimatedPoseInterpolator.getSample(timestamp);
		return estimatedPoseAtTimestamp.orElseGet(wpiPoseEstimator::getEstimatedPosition);
	}

	@Override
	public void updateOdometry(OdometryObservation[] odometryObservations) {
		for(OdometryObservation odometryObservation : odometryObservations) {
			addOdometryObservation(odometryObservation);
		}
	}

	private void addOdometryObservation(OdometryObservation observation) {
		wpiPoseEstimator.update(observation.gyroAngle(), observation.wheelPositions());
		Twist2d twist = lastOdometryValues.kinematics().toTwist2d(lastOdometryValues.wheelPositions(), observation.wheelPositions());
		twist = PoseEstimationMath.addGyroToTwist(twist, observation.gyroAngle(), lastOdometryValues.gyroAngle());
		lastOdometryValues = new OdometryValues(lastOdometryValues.kinematics(), observation.wheelPositions(), observation.gyroAngle());
		odometryPose = odometryPose.exp(twist);
	}

	@Override
	public void resetOdometry(SwerveModulePosition[] wheelPositions, Rotation2d gyroAngle, Pose2d robotPose) {
		wpiPoseEstimator.resetPosition(gyroAngle, wheelPositions, robotPose);
		odometryPose = robotPose;
	}

	@Override
	public Pose2d getOdometryPose() {
		return odometryPose;
	}

	@Override
	public void setHeading(Rotation2d newHeading) {
		wpiPoseEstimator.resetRotation(newHeading);
		odometryPose = new Pose2d(odometryPose.getTranslation(), newHeading);
	}

	@Override
	public void updateVision(List<VisionObservation> visionObservations) {
		for(VisionObservation observation : visionObservations) {
			addVisionObservation(observation);
		}
	}

	private void addVisionObservation(VisionObservation observation) {
		wpiPoseEstimator.addVisionMeasurement(
			observation.robotPose(),
			observation.timestamp(),
			standardDeviationsToMatrix(observation.standardDeviations())
		);
		lastVisionPose = observation.robotPose();
	}

	private Matrix<N3, N1> standardDeviationsToMatrix(double[] standardDeviation) {
		return VecBuilder.fill(
			standardDeviation[PoseArrayEntryValue.X_VALUE.getEntryValue()],
			standardDeviation[PoseArrayEntryValue.Y_VALUE.getEntryValue()],
			standardDeviation[PoseArrayEntryValue.ROTATION_VALUE.getEntryValue()]
		);
	}

	@Override
	public Optional<Pose2d> getVisionPose() {
		return Optional.of(lastVisionPose);
	}

	@Override
	public void subsystemPeriodic() {
		estimatedPoseInterpolator.addSample(TimeUtils.getCurrentTimeSeconds(), wpiPoseEstimator.getEstimatedPosition());
		updateVision(visionFilterer.getFilteredVisionObservations());
	}

}
