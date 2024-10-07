package frc.robot.poseestimator;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveWheelPositions;
import frc.robot.subsystems.GBSubsystem;
import frc.robot.vision.limelights.GyroAngleValues;
import frc.robot.vision.limelights.ILimelightFilterer;
import frc.robot.poseestimator.observations.OdometryObservation;
import frc.robot.poseestimator.observations.VisionObservation;
import frc.utils.Conversions;
import org.littletonrobotics.junction.Logger;
import java.util.List;
import java.util.NoSuchElementException;
import java.util.Optional;

public class GBPoseEstimator extends GBSubsystem implements IPoseEstimator {

	private final TimeInterpolatableBuffer<Pose2d> odometryPoseInterpolator;
	private final TimeInterpolatableBuffer<Pose2d> estimatedPoseInterpolator;
	private final SwerveDriveKinematics kinematics;
	private final double[] odometryStandardDeviations;
	private Pose2d odometryPose;
	private Pose2d estimatedPose;
	private SwerveDriveWheelPositions latestWheelPositions;
	private Rotation2d latestGyroAngle;
	private final ILimelightFilterer limelightFilterer;

	public GBPoseEstimator(
		String logPath,
		SwerveDriveKinematics kinematics,
		SwerveDriveWheelPositions initialWheelPositions,
		Rotation2d initialGyroAngle,
		double[] odometryStandardDeviations,
		ILimelightFilterer limelightFilterer,
		Pose2d initialRobotPose
	) {
		super(logPath);
		this.odometryPose = new Pose2d();
		this.estimatedPose = new Pose2d();
		this.odometryPoseInterpolator = TimeInterpolatableBuffer.createBuffer(PoseEstimatorConstants.POSE_BUFFER_SIZE_SECONDS);
		this.estimatedPoseInterpolator = TimeInterpolatableBuffer.createBuffer(PoseEstimatorConstants.POSE_BUFFER_SIZE_SECONDS);
		this.kinematics = kinematics;
		this.latestWheelPositions = initialWheelPositions;
		this.latestGyroAngle = initialGyroAngle;
		this.odometryStandardDeviations = new double[PoseArrayEntryValue.POSE_ARRAY_LENGTH];
		this.limelightFilterer = limelightFilterer;
		this.limelightFilterer.setEstimatedPoseAtTimestampFunction(this::getEstimatedPoseAtTimeStamp);
		setOdometryStandardDeviations(odometryStandardDeviations);
		resetPose(initialRobotPose);
	}

	public ILimelightFilterer getLimelightFilterer() {
		return limelightFilterer;
	}

	public void resetPoseByLimelight() {
		getVisionPose().ifPresent(this::resetPose);
	}

	@Override
	public void setOdometryStandardDeviations(double[] newStandardDeviations) {
		for (int i = 0; i < newStandardDeviations.length; i++) {
			odometryStandardDeviations[i] = newStandardDeviations[i] * newStandardDeviations[i];
		}
	}

	@Override
	public void resetPose(Pose2d initialPose) {
		this.estimatedPose = initialPose;
		this.latestGyroAngle = initialPose.getRotation();
		this.odometryPose = initialPose;
		odometryPoseInterpolator.clear();
	}

	@Override
	public void resetOdometry(SwerveDriveWheelPositions wheelPositions, Rotation2d gyroAngle, Pose2d robotPose) {
		this.latestWheelPositions = wheelPositions;
		this.latestGyroAngle = gyroAngle;
		this.odometryPose = robotPose;
		odometryPoseInterpolator.clear();
	}


	@Override
	public Pose2d getOdometryPose() {
		return odometryPose;
	}

	@Override
	public Optional<Pose2d> getVisionPose() {
		List<VisionObservation> stackedRawData = limelightFilterer.getAllAvailableLimelightRawData();
		List<VisionObservation> rawData = limelightFilterer.getAllAvailableLimelightRawData();
		while (stackedRawData.size() < PoseEstimatorConstants.VISION_OBSERVATION_COUNT_FOR_AVERAGED_POSE_CALCULATION && !rawData.isEmpty()) {
			if (!stackedRawData.contains(rawData.get(0))) {
				stackedRawData.addAll(rawData);
			}
			rawData = limelightFilterer.getAllAvailableLimelightRawData();
		}
		Pose2d averagePose = PoseEstimationMath.weightedPoseMean(stackedRawData);
		Pose2d visionPose = new Pose2d(averagePose.getX(), averagePose.getY(), odometryPose.getRotation());
		return Optional.of(visionPose);
	}

	@Override
	public Pose2d getEstimatedPose() {
		return estimatedPose;
	}

	@Override
	public Pose2d getEstimatedPoseAtTimeStamp(double timeStamp) {
		Optional<Pose2d> estimatedPoseAtTimestamp = estimatedPoseInterpolator.getSample(timeStamp);
		return estimatedPoseAtTimestamp.orElseGet(() -> estimatedPose);
	}


	@Override
	public void updateVision(List<VisionObservation> visionObservations) {
		for (VisionObservation visionObservation : visionObservations) {
			if (!isObservationTooOld(visionObservation)) {
				addVisionObservation(visionObservation);
			}
		}
	}

	@Override
	public void updateOdometry(List<OdometryObservation> odometryObservation) {
		for (OdometryObservation observation : odometryObservation) {
			addOdometryObservation(observation);
		}
		logEstimatedPose();
	}

	@Override
	public void updatePoseEstimator(List<OdometryObservation> odometryObservation, List<VisionObservation> visionObservations) {
		updateOdometry(odometryObservation);
		updateVision(visionObservations);
	}

	private boolean isObservationTooOld(VisionObservation visionObservation) {
		try {
			return odometryPoseInterpolator.getInternalBuffer().lastKey() - PoseEstimatorConstants.POSE_BUFFER_SIZE_SECONDS
				> visionObservation.timestamp();
		} catch (NoSuchElementException ignored) {
			return true;
		}
	}

	private void addVisionObservation(VisionObservation observation) {
		Optional<Pose2d> odometryInterpolatedPoseSample = odometryPoseInterpolator.getSample(observation.timestamp());
		odometryInterpolatedPoseSample.ifPresent(odometryPoseSample -> {
			Pose2d currentEstimation = PoseEstimationMath
				.combineVisionToOdometry(observation, odometryPoseSample, estimatedPose, odometryPose, odometryStandardDeviations);
			estimatedPose = new Pose2d(currentEstimation.getTranslation(), odometryPoseSample.getRotation());
			estimatedPoseInterpolator.addSample(Conversions.microSecondsToSeconds(Logger.getRealTimestamp()), estimatedPose);
		});
	}

	private void addOdometryObservation(OdometryObservation observation) {
		updateGyroAnglesInLimeLight(observation.gyroAngle());
		Twist2d twist = kinematics.toTwist2d(latestWheelPositions, observation.wheelsPositions());
		twist = PoseEstimationMath.addGyroToTwist(twist, observation.gyroAngle(), latestGyroAngle);
		latestGyroAngle = observation.gyroAngle();
		latestWheelPositions = observation.wheelsPositions();
		odometryPose = odometryPose.exp(twist);
		estimatedPose = estimatedPose.exp(twist);
		odometryPoseInterpolator.addSample(observation.timestamp(), odometryPose);
	}

	private void updateGyroAnglesInLimeLight(Rotation2d gyroAngle) {
		if (gyroAngle != null) {
			limelightFilterer.updateGyroAngles(new GyroAngleValues(gyroAngle.getDegrees(), 0, 0, 0, 0, 0));
		}
	}

	public void logEstimatedPose() {
		Logger.recordOutput(super.getLogPath() + "EstimatedPose", getEstimatedPose());
	}

	@Override
	public void subsystemPeriodic() {
		if (!limelightFilterer.isPoseEstimationCorrect()) {
			resetPoseByLimelight();
		}
		updateVision(limelightFilterer.getFilteredVisionObservations());
	}

}
