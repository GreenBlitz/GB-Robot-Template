package frc.robot.poseestimator;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveWheelPositions;
import frc.robot.vision.limelights.GyroAngleValues;
import frc.robot.vision.limelights.LimeLightConstants;
import frc.robot.vision.limelights.LimelightFilterer;
import frc.robot.poseestimator.observations.OdometryObservation;
import frc.robot.poseestimator.observations.VisionObservation;
import org.littletonrobotics.junction.Logger;
import java.util.List;
import java.util.NoSuchElementException;
import java.util.Optional;

public class GBPoseEstimator implements IPoseEstimator {

	private final TimeInterpolatableBuffer<Pose2d> odometryPoseInterpolator;
	private final TimeInterpolatableBuffer<Pose2d> estimatedPoseInterpolator;
	private final SwerveDriveKinematics kinematics;
	private final double[] odometryStandardDeviations;
	private Pose2d odometryPose;
	private Pose2d estimatedPose;
	private SwerveDriveWheelPositions lastWheelPositions;
	private Rotation2d lastGyroAngle;
	private VisionObservation lastVisionObservation;
	private final LimelightFilterer limelightFilterer;

	public GBPoseEstimator(
		SwerveDriveKinematics kinematics,
		SwerveDriveWheelPositions initialWheelPositions,
		Rotation2d initialGyroAngle,
		double[] odometryStandardDeviations
	) {
		this.odometryPose = new Pose2d();
		this.estimatedPose = new Pose2d();
		this.odometryPoseInterpolator = TimeInterpolatableBuffer.createBuffer(PoseEstimatorConstants.POSE_BUFFER_SIZE_SECONDS);
		this.estimatedPoseInterpolator = TimeInterpolatableBuffer.createBuffer(PoseEstimatorConstants.POSE_BUFFER_SIZE_SECONDS);
		this.kinematics = kinematics;
		this.lastWheelPositions = initialWheelPositions;
		this.lastGyroAngle = initialGyroAngle;
		this.odometryStandardDeviations = new double[3];
		this.limelightFilterer = new LimelightFilterer(LimeLightConstants.DEFAULT_CONFIG, this);
		setOdometryStandardDeviations(odometryStandardDeviations);
	}

	public LimelightFilterer getLimelightFilterer() {
		return limelightFilterer;
	}

	public void resetPoseByLimelight() {
		List<VisionObservation> stackedRawData = limelightFilterer.getAllAvailableLimelightData();
		List<VisionObservation> rawData;
		while (
			stackedRawData.size() < PoseEstimatorConstants.OBSERVATION_COUNT_FOR_POSE_CALIBRATION
				&& !(rawData = limelightFilterer.getAllAvailableLimelightData()).isEmpty()
		) {
			if (!stackedRawData.contains(rawData.get(0))) {
				stackedRawData.addAll(rawData);
			}
		}
		Pose2d pose2d = weightedPoseMean(stackedRawData);
		resetPose(new Pose2d(pose2d.getX(), pose2d.getY(), odometryPose.getRotation()));
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
		this.lastGyroAngle = initialPose.getRotation();
		this.odometryPose = initialPose;
		odometryPoseInterpolator.clear();
	}

	@Override
	public void resetOdometry(SwerveDriveWheelPositions wheelPositions, Rotation2d gyroAngle, Pose2d pose) {
		this.lastWheelPositions = wheelPositions;
		this.lastGyroAngle = gyroAngle;
		this.odometryPose = pose;
		odometryPoseInterpolator.clear();
	}


	@Override
	public Pose2d getOdometryPose() {
		return odometryPose;
	}

	@Override
	public Optional<Pose2d> getVisionPose() {
		return Optional.of(lastVisionObservation.visionPose());
	}

	@Override
	public Pose2d getEstimatedPose() {
		return estimatedPose;
	}

	@Override
	public Optional<Pose2d> getEstimatedPoseAtTimeStamp(double timeStamp) {
		return estimatedPoseInterpolator.getSample(timeStamp);
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
			logEstimatedPose();
		}
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
		this.lastVisionObservation = observation;
		Optional<Pose2d> odometryInterpolatedPoseSample = odometryPoseInterpolator.getSample(observation.timestamp());
		odometryInterpolatedPoseSample.ifPresent(odometryPoseSample -> {
			estimatedPose = PoseEstimationMath.combineVisionToOdometry(
				observation,
				odometryPoseSample,
				estimatedPose,
				odometryPose,
				odometryStandardDeviations
			);
			estimatedPoseInterpolator.addSample(observation.timestamp(), estimatedPose);
		});
	}

	private void addOdometryObservation(OdometryObservation observation) {
		updateGyroAnglesInLimeLight(observation.gyroAngle());
		Twist2d twist = kinematics.toTwist2d(lastWheelPositions, observation.wheelsPositions());
		twist = PoseEstimationMath.addGyroToTwist(twist, observation.gyroAngle(), lastGyroAngle);
		lastGyroAngle = observation.gyroAngle();
		lastWheelPositions = observation.wheelsPositions();
		odometryPose = odometryPose.exp(twist);
		estimatedPose = estimatedPose.exp(twist);
		odometryPoseInterpolator.addSample(observation.timestamp(), odometryPose);
	}

	private void updateGyroAnglesInLimeLight(Rotation2d gyroAngles) {
		if (gyroAngles != null) {
			limelightFilterer.updateGyroAngles(new GyroAngleValues(gyroAngles.getDegrees(), 0, 0, 0, 0, 0));
		}
	}

	private Pose2d weightedPoseMean(List<VisionObservation> observations) {
		Pose2d output = new Pose2d();
		double positionDeviationSum = 0;
		double rotationDeviationSum = 0;

		for (VisionObservation observation : observations) {
			double positionWeight = Math.pow(observation.standardDeviations()[PoseArrayEntryValue.X_VALUE.getEntryValue()], -1);
			double rotationWeight = Math.pow(observation.standardDeviations()[PoseArrayEntryValue.X_VALUE.getEntryValue()], -1);
			positionDeviationSum += positionWeight;
			rotationDeviationSum += rotationWeight;
			output = new Pose2d(
				output.getX() + observation.visionPose().getX() * positionWeight,
				output.getY() + observation.visionPose().getY() * positionWeight,
				output.getRotation().plus(observation.visionPose().getRotation()).times(rotationWeight)
			);
		}

		output = new Pose2d(output.getTranslation().div(positionDeviationSum), output.getRotation().div(rotationDeviationSum));

		return output;
	}

	public void logEstimatedPose() {
		Logger.recordOutput(PoseEstimatorConstants.LOG_PATH + "EstimatedPose", getEstimatedPose());
	}

}
