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
import org.littletonrobotics.junction.Logger;
import java.util.List;
import java.util.NoSuchElementException;
import java.util.Optional;
import java.util.function.Consumer;

public class GBPoseEstimator extends GBSubsystem implements IPoseEstimator {

	private final TimeInterpolatableBuffer<Pose2d> odometryPoseInterpolator;
	private final TimeInterpolatableBuffer<Pose2d> estimatedPoseInterpolator;
	private final ILimelightFilterer limelightFilterer;
	private final SwerveDriveKinematics kinematics;
	private final double[] odometryStandardDeviations;
	private Pose2d odometryPose;
	private Pose2d estimatedPose;
	private SwerveDriveWheelPositions latestWheelPositions;
	private Rotation2d latestGyroAngle;
	private Rotation2d headingOffset;
	private Consumer<Rotation2d> resetSwerve;


	public GBPoseEstimator(
		Consumer<Rotation2d> resetSwerve,
		String logPath,
		ILimelightFilterer limelightFilterer,
		SwerveDriveKinematics kinematics,
		SwerveDriveWheelPositions initialWheelPositions,
		Rotation2d initialGyroAngle,
		double[] odometryStandardDeviations,
		Pose2d initialRobotPose
	) {
		super(logPath);
		this.resetSwerve = resetSwerve;
		this.odometryPose = new Pose2d();
		this.estimatedPose = new Pose2d();
		this.odometryPoseInterpolator = TimeInterpolatableBuffer.createBuffer(PoseEstimatorConstants.POSE_BUFFER_SIZE_SECONDS);
		this.estimatedPoseInterpolator = TimeInterpolatableBuffer.createBuffer(PoseEstimatorConstants.POSE_BUFFER_SIZE_SECONDS);
		this.limelightFilterer = limelightFilterer;
		this.kinematics = kinematics;
		this.latestWheelPositions = initialWheelPositions;
		this.latestGyroAngle = initialGyroAngle;
		this.odometryStandardDeviations = new double[PoseArrayEntryValue.POSE_ARRAY_LENGTH];
		this.limelightFilterer.setEstimatedPoseAtTimestampFunction(this::getEstimatedPoseAtTimeStamp);
		setOdometryStandardDeviations(odometryStandardDeviations);
		resetPose(initialRobotPose);
//		calculateHeadingOffset(initialGyroAngle);
	}

	public ILimelightFilterer getLimelightFilterer() {
		return limelightFilterer;
	}

	public void resetPoseByLimelight() {
		getVisionPose().ifPresent(this::resetPose);
	}

	public void calculateHeadingOffset(Rotation2d gyroAngle) {
		headingOffset = gyroAngle;
	}

	@Override
	public void resetHeadingOffset(Rotation2d newHeading) {
		if (latestGyroAngle != null) {
			headingOffset = newHeading.minus(latestGyroAngle);
		}
	}

	private Rotation2d getEstimatedRobotHeadingByVision() {
		if (true) {
			return limelightFilterer.getAllRobotHeadingEstimations().get(0);
		}
		List<Rotation2d> stackedHeadingEstimations = limelightFilterer.getAllRobotHeadingEstimations();
		List<Rotation2d> headingEstimation = limelightFilterer.getAllRobotHeadingEstimations();
		while (
			stackedHeadingEstimations.size() < PoseEstimatorConstants.VISION_OBSERVATION_COUNT_FOR_AVERAGED_POSE_CALCULATION
				&& !headingEstimation.isEmpty()
		) {
			if (!stackedHeadingEstimations.contains(headingEstimation.get(0))) {
				stackedHeadingEstimations.addAll(headingEstimation);
			}
			headingEstimation = limelightFilterer.getAllRobotHeadingEstimations();
		}
		return PoseEstimationMath.calculateAngleAverage(stackedHeadingEstimations);
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
		this.resetSwerve.accept(initialPose.getRotation());
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
//			if (!isObservationTooOld(visionObservation)) {
			if (true) {
				addVisionObservation(visionObservation);
			}
		}
		logEstimatedPose();
	}

	@Override
	public void updateOdometry(List<OdometryObservation> odometryObservations) {
		for (OdometryObservation observation : odometryObservations) {
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
		Logger.recordOutput("inside vision", observation.timestamp());
		Optional<Pose2d> odometryInterpolatedPoseSample = odometryPoseInterpolator.getSample(observation.timestamp());
		odometryInterpolatedPoseSample.ifPresent(odometryPoseSample -> {
			Pose2d currentEstimation = PoseEstimationMath
				.combineVisionToOdometry(observation, odometryPoseSample, estimatedPose, odometryPose, odometryStandardDeviations);
			estimatedPose = new Pose2d(currentEstimation.getTranslation(), odometryPoseSample.getRotation());
//			estimatedPoseInterpolator.addSample(Conversions.microSecondsToSeconds(Logger.getRealTimestamp()), estimatedPose);
			estimatedPoseInterpolator.addSample(Logger.getRealTimestamp() / 1.0e6, estimatedPose);
		});
	}

	private void addOdometryObservation(OdometryObservation observation) {
//		updateGyroAnglesInLimeLight(observation.gyroAngle());
		Logger.recordOutput("inside odometry", observation.timestamp());
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
			Rotation2d headingWithOffset = gyroAngle.plus(headingOffset);
			limelightFilterer.updateGyroAngles(new GyroAngleValues(headingWithOffset.getDegrees(), 0, 0, 0, 0, 0));
		}
	}

	public void logEstimatedPose() {
		Logger.recordOutput(super.getLogPath() + "EstimatedPose", getEstimatedPose());
	}

	@Override
	public void subsystemPeriodic() {
		if (!limelightFilterer.isPoseEstimationCorrect()) {
//			resetPoseByLimelight();
		}
	}

}
