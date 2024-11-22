package frc.robot.poseestimator.helpers;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import frc.robot.poseestimator.PoseEstimationMath;
import frc.robot.poseestimator.observations.VisionObservation;
import frc.utils.time.TimeUtils;
import org.littletonrobotics.junction.Logger;

import java.util.*;

public class VisionDenoiser {

	private final int maximumSize;
	private final ArrayList<VisionObservation> observations;
	private final String logPath;

	private final LinearFilter xFilter;
	private final LinearFilter yFilter;
	private final LinearFilter angleFilterRadians;
	private Pose2d lastFilterOutput;

	private final LinearFilter xFilterWithOdometryFix;
	private final LinearFilter yFilterWithOdometryFix;
	private final LinearFilter angleFilterRadiansWithOdometryFix;


	public VisionDenoiser(int maximumSize, String logPath) {
		this.maximumSize = maximumSize;
		this.observations = new ArrayList<>();
		this.logPath = logPath + "/";

		this.xFilter = LinearFilter.movingAverage(maximumSize);
		this.yFilter = LinearFilter.movingAverage(maximumSize);
		this.angleFilterRadians = LinearFilter.movingAverage(maximumSize);

		this.xFilterWithOdometryFix = LinearFilter.movingAverage(maximumSize);
		this.yFilterWithOdometryFix = LinearFilter.movingAverage(maximumSize);
		this.angleFilterRadiansWithOdometryFix = LinearFilter.movingAverage(maximumSize);
	}

	private void popLastObservationIfQueueIsTooLarge() {
		if (observations.size() > maximumSize) {
			observations.remove(0);
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
		for (int i = 0; i < observations.size(); i++) {
			Logger.recordOutput(logPath + "innerObservation/" + i, observations.get(i).robotPose());
		}
	}

	private ArrayList<Pose2d> getRobotPosesFromGivenObservations() {
		ArrayList<Pose2d> output = new ArrayList<>();
		for (VisionObservation observation : observations) {
			output.add(observation.robotPose());
		}
		return output;
	}

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

	private ArrayList<Pose2d> fixAccordingToOdometry(Pose2d odometryPose, TimeInterpolatableBuffer<Pose2d> odometryPoseInterpolator) {
		ArrayList<Pose2d> output = new ArrayList<>();
		int observationCount = 0;

		for (VisionObservation observation : observations) {
			observationCount++;
			double observationTime = observation.timestamp();
			Optional<Pose2d> odometryFix = odometryPoseInterpolator.getSample(observationTime);
			if (odometryFix.isPresent()) {
				Transform2d sampleDifferenceFromPose = new Transform2d(odometryFix.get(), odometryPose);
				Pose2d fixedPose = observation.robotPose().plus(sampleDifferenceFromPose);
				Logger.recordOutput(logPath + "fixedObservation/" + observationCount, fixedPose);
				output.add(fixedPose);
			}
			;
		}
		return output;
	}
	// @formatter:on

	private Pose2d getFilterResult(double x, double y, double ang) {
		return new Pose2d(x, y, Rotation2d.fromRadians(ang));
	}

	public Optional<VisionObservation> calculateFixedObservationByOdometryLinearFilter(
		Pose2d odometryPose,
		TimeInterpolatableBuffer<Pose2d> odometryPoseInterpolator
	) {
		if (observations.size() != maximumSize) {
			return Optional.empty();
		}

		VisionObservation calculatedFixedPoseByOdometryLinearFilter = calculateFixedPoseByOdometryLinearFilter(
			odometryPose,
			odometryPoseInterpolator
		);
		Logger.recordOutput(logPath + "stdDevs", calculatedFixedPoseByOdometryLinearFilter.standardDeviations());
		return Optional.of(
			new VisionObservation(
				calculatedFixedPoseByOdometryLinearFilter.robotPose(),
				calculatedFixedPoseByOdometryLinearFilter.standardDeviations(),
				TimeUtils.getCurrentTimeSeconds()
			)
		);
	}

	private VisionObservation calculateFixedPoseByOdometryLinearFilter(
		Pose2d odometryPose,
		TimeInterpolatableBuffer<Pose2d> odometryPoseInterpolator
	) {
		ArrayList<Pose2d> fixedData = fixAccordingToOdometry(odometryPose, odometryPoseInterpolator);
		Pose2d lastObservationPose = fixedData.get(fixedData.size() - 1);

		xFilterWithOdometryFix.reset(fixedData.stream().mapToDouble(Pose2d::getX).toArray(), new double[] {});
		yFilterWithOdometryFix.reset(fixedData.stream().mapToDouble(Pose2d::getY).toArray(), new double[] {});
		angleFilterRadiansWithOdometryFix.reset(
			fixedData.stream().mapToDouble((Pose2d visionObservation) -> visionObservation.getRotation().getRadians()).toArray(),
			new double[] {}
		);

		return new VisionObservation(
			new Pose2d(
				xFilterWithOdometryFix.calculate(lastObservationPose.getX()),
				yFilterWithOdometryFix.calculate(lastObservationPose.getY()),
				Rotation2d.fromRadians(angleFilterRadiansWithOdometryFix.calculate(lastObservationPose.getRotation().getRadians()))
			),
			PoseEstimationMath.calculateStandardDeviationOfPose(fixedData),
			TimeUtils.getCurrentTimeSeconds()
		);
	}

	public Optional<VisionObservation> calculateLinearFilterResult() {
		if (observations.isEmpty()) {
			return Optional.empty();
		}
		double timestamp = getLastObservation(observations).timestamp();
		return Optional.of(new VisionObservation(lastFilterOutput,
				PoseEstimationMath.calculateStandardDeviationOfPose(getRobotPosesFromGivenObservations()),
			timestamp
		));
	}

}
