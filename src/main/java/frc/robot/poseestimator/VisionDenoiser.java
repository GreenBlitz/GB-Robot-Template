package frc.robot.poseestimator;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import frc.robot.poseestimator.observations.VisionObservation;
import frc.utils.time.TimeUtils;

import java.util.*;

public class VisionDenoiser {

	private final int maximumSize;
	private LinkedList<VisionObservation> observations;

	public VisionDenoiser(int maximumSize) {
		this.maximumSize = maximumSize;
	}

	private void popLastIfQueueTooLarge() {
		if (observations.size() > maximumSize) {
			observations.poll();
		}
	}

	public void addVisionObservation(VisionObservation observation) {
		observations.add(observation);
		popLastIfQueueTooLarge();
	}

	public ArrayList<Pose2d> getRobotPoseObservations() {
		ArrayList<Pose2d> output = new ArrayList<>();
		for (VisionObservation observation : observations) {
			output.add(observation.robotPose());
		}
		return output;
	}

	private Optional<VisionObservation> calculateAverage(LinkedList<VisionObservation> overwrittenObservations) {
		if (!overwrittenObservations.isEmpty()) {
			return Optional.empty();
		}
		ArrayList<Pose2d> poses = getRobotPoseObservations();
		return Optional.of(
			new VisionObservation(
				PoseEstimationMath.meanOfPose(poses),
				PoseEstimationMath.calculateStandardDeviationOfPose(poses),
				overwrittenObservations.peekLast().timestamp()
			)
		);
	}

	public Optional<VisionObservation> calculateAverage() {
		return calculateAverage(observations);
	}

	private Optional<VisionObservation> calculateWeightedAverage(LinkedList<VisionObservation> overwrittenObservations) {
		if (!overwrittenObservations.isEmpty()) {
			return Optional.empty();
		}
		ArrayList<Pose2d> poses = getRobotPoseObservations();
		return Optional.of(
			new VisionObservation(
				PoseEstimationMath.weightedPoseMean(overwrittenObservations),
				PoseEstimationMath.calculateStandardDeviationOfPose(poses),
				overwrittenObservations.peekLast().timestamp()
			)
		);
	}

	public Optional<VisionObservation> calculateWeightedAverage() {
		return calculateWeightedAverage(observations);
	}

	private LinkedList<VisionObservation>
		fixAccordingToOdometry(Pose2d currentPose, Pose2d odometryPose, TimeInterpolatableBuffer<Pose2d> odometryPoseInterpolator) {
		LinkedList<VisionObservation> output = new LinkedList<>();
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
				output.add(
					new VisionObservation(
						fixedPose,
						PoseEstimationMath.calculateStandardDeviationOfPose(new ArrayList<>(List.of(fixedPose, currentPose))),
						currentTimestamp
					)
				);
			});
		}
		return output;
	}

	public Optional<VisionObservation>
		calculateAverageFixedPose(Pose2d currentPose, Pose2d odometryPose, TimeInterpolatableBuffer<Pose2d> odometryPoseInterpolator) {
		return calculateAverage(fixAccordingToOdometry(currentPose, odometryPose, odometryPoseInterpolator));
	}

	public Optional<VisionObservation>
		calculateWeightedAverageFixedPose(Pose2d currentPose, Pose2d odometryPose, TimeInterpolatableBuffer<Pose2d> odometryPoseInterpolator) {
		return calculateWeightedAverage(fixAccordingToOdometry(currentPose, odometryPose, odometryPoseInterpolator));
	}


}
