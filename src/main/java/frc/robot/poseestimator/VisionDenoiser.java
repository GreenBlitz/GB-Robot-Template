package frc.robot.poseestimator;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import frc.robot.poseestimator.observations.VisionObservation;
import frc.utils.time.TimeUtils;

import java.util.*;

public class VisionDenoiser {

	private final int maximumSize;
	private ArrayList<VisionObservation> observations;
	private LinearFilter xFilter;
	private LinearFilter yFilter;
	private LinearFilter angleFilterRadians;
	private Pose2d lastFilterOutput;

	public VisionDenoiser(int maximumSize) {
		this.maximumSize = maximumSize;
		this.observations = new ArrayList<>();

		this.xFilter = LinearFilter.movingAverage(maximumSize);
		this.yFilter = LinearFilter.movingAverage(maximumSize);
		this.angleFilterRadians = LinearFilter.movingAverage(maximumSize);
	}

	private void popLastObservationIfQueueIsTooLarge() {
		if (observations.size() > maximumSize) {
			observations.remove(observations.size() - 1);
		}
	}

	private VisionObservation getLastObservation(List<VisionObservation> list) {
		return list.get(list.size() - 1);
	}

	public void addVisionObservation(VisionObservation observation) {
		observations.add(observation);
		popLastObservationIfQueueIsTooLarge();
		lastFilterOutput = getFilterResult(
			xFilter.calculate(observation.robotPose().getX()),
			yFilter.calculate(observation.robotPose().getY()),
			angleFilterRadians.calculate(observation.robotPose().getRotation().getRadians())
		);
	}

	private ArrayList<Pose2d> getRobotPosesFromGivenObservations() {
		ArrayList<Pose2d> output = new ArrayList<>();
		for (VisionObservation observation : observations) {
			output.add(observation.robotPose());
		}
		return output;
	}

	private Optional<VisionObservation> calculateAverage(List<VisionObservation> overwrittenObservations) {
		if (!overwrittenObservations.isEmpty()) {
			return Optional.empty();
		}
		List<Pose2d> poses = getRobotPosesFromGivenObservations();
		return Optional.of(
			new VisionObservation(
				PoseEstimationMath.meanOfPose(poses),
				PoseEstimationMath.calculateStandardDeviationOfPose(poses),
				getLastObservation(overwrittenObservations).timestamp()
			)
		);
	}

	public Optional<VisionObservation> calculateAverage() {
		return calculateAverage(observations);
	}

	// @formatter:off
	public Optional<VisionObservation> calculateAverageFixedPose(
		Pose2d currentPose,
		Pose2d odometryPose,
		TimeInterpolatableBuffer<Pose2d> odometryPoseInterpolator
	) {
		return calculateAverage(fixAccordingToOdometry(currentPose, odometryPose, odometryPoseInterpolator));
	}
	// @formatter:on

	private Optional<VisionObservation> calculateWeightedAverage(List<VisionObservation> overwrittenObservations) {
		if (!overwrittenObservations.isEmpty()) {
			return Optional.empty();
		}
		List<Pose2d> poses = getRobotPosesFromGivenObservations();
		return Optional.of(
			new VisionObservation(
				PoseEstimationMath.weightedPoseMean(overwrittenObservations),
				PoseEstimationMath.calculateStandardDeviationOfPose(poses),
				getLastObservation(overwrittenObservations).timestamp()
			)
		);
	}

	public Optional<VisionObservation> calculateWeightedAverage() {
		return calculateWeightedAverage(observations);
	}

	// @formatter:off
	public Optional<VisionObservation> calculateWeightedAverageFixedPose(
		Pose2d currentEstimatedPose,
		Pose2d odometryPose,
		TimeInterpolatableBuffer<Pose2d> odometryPoseInterpolator
	) {
		return calculateWeightedAverage(fixAccordingToOdometry(currentEstimatedPose, odometryPose, odometryPoseInterpolator));
	}

	private List<VisionObservation> fixAccordingToOdometry(
		Pose2d currentPose,
		Pose2d odometryPose,
		TimeInterpolatableBuffer<Pose2d>
		odometryPoseInterpolator
	) {
		ArrayList<VisionObservation> output = new ArrayList<>();
		ArrayList<Pose2d> pastFixedPoses = new ArrayList<>();
		pastFixedPoses.add(currentPose);

		for (VisionObservation observation : observations) {
			double currentTimestamp = TimeUtils.getCurrentTimeSeconds();
			Optional<Pose2d> odometryFix = odometryPoseInterpolator.getSample(currentTimestamp);
			odometryFix.ifPresent(odometryPoseSample -> {
				Pose2d currentEstimation = PoseEstimationMath.combineVisionToOdometry(
					observation,
					odometryPoseSample,
					currentPose,
					odometryPose,
					PoseEstimatorConstants.DEFAULT_ODOMETRY_STANDARD_DEVIATIONS
				);
				Pose2d fixedPose = new Pose2d(currentEstimation.getTranslation(), odometryPoseSample.getRotation());
				pastFixedPoses.add(fixedPose);
				output.add(
					new VisionObservation(
						fixedPose,
						PoseEstimationMath.calculateStandardDeviationOfPose(pastFixedPoses),
						currentTimestamp
					)
				);
			});
		}
		return output;
	}
	// @formatter:on

	private Pose2d getFilterResult(double x, double y, double ang) {
		return new Pose2d(x, y, Rotation2d.fromRadians(ang));
	}

	public Optional<VisionObservation> calculateLinearFilterResult() {
		if (observations.size() == 0) {
			return Optional.empty();
		}
		double timestamp = getLastObservation(observations).timestamp();
		return Optional.of(
			new VisionObservation(
				lastFilterOutput,
				PoseEstimationMath.calculateStandardDeviationOfPose(getRobotPosesFromGivenObservations()),
				timestamp
			)
		);
	}

}
