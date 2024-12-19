package frc.robot.poseestimator.helpers;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import frc.robot.poseestimator.PoseEstimationMath;
import frc.robot.vision.rawdata.VisionData;
import frc.utils.time.TimeUtils;
import org.littletonrobotics.junction.Logger;

import java.util.*;

public class VisionDenoiser {

	private final int maximumSize;
	private final ArrayList<VisionData> observations;
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

	private VisionData getLastObservation(List<VisionData> list) {
		return list.get(list.size() - 1);
	}

	public void addVisionObservation(VisionData observation) {
		observations.add(observation);
		popLastObservationIfQueueIsTooLarge();
		lastFilterOutput = getFilterResult(
			xFilter.calculate(observation.getEstimatedPose().getX()),
			yFilter.calculate(observation.getEstimatedPose().getY()),
			angleFilterRadians.calculate(observation.getEstimatedPose().getRotation().toRotation2d().getRadians())
		);
		for (int i = 0; i < observations.size(); i++) {
			Logger.recordOutput(logPath + "innerObservation/" + i, observations.get(i).getEstimatedPose());
		}
	}

	private ArrayList<Pose2d> getRobotPosesFromGivenObservations() {
		ArrayList<Pose2d> output = new ArrayList<>();
		for (VisionData observation : observations) {
			output.add(observation.getEstimatedPose().toPose2d());
		}
		return output;
	}

	private Optional<ProcessedVisionData> calculateWeightedAverage(List<VisionData> overwrittenObservations, Pose2d referencePose) {
		if (!overwrittenObservations.isEmpty()) {
			return Optional.empty();
		}
		List<Pose2d> poses = getRobotPosesFromGivenObservations();
		return Optional.of(
			new ProcessedVisionData(
				PoseEstimationMath.weightedPoseMean(
					overwrittenObservations.stream()
						.map((VisionData visionData) -> PoseEstimationMath.processVisionData(visionData, referencePose))
						.toList()
				),
				getLastObservation(overwrittenObservations).getTimestamp(),
				PoseEstimationMath.calculateStandardDeviationOfPose(poses)
			)
		);
	}

	public Optional<ProcessedVisionData> calculateWeightedAverage(Pose2d referencePose) {
		return calculateWeightedAverage(observations, referencePose);
	}

	private ArrayList<Pose2d> fixAccordingToOdometry(Pose2d odometryPose, TimeInterpolatableBuffer<Pose2d> odometryPoseInterpolator) {
		ArrayList<Pose2d> output = new ArrayList<>();
		int observationCount = 0;
		ArrayList<Pose2d> odometryPoses = new ArrayList<>();

		for (VisionData observation : observations) {
			observationCount++;
			double observationTime = observation.getTimestamp();
			Optional<Pose2d> odometryFix = odometryPoseInterpolator.getSample(observationTime);
			if (odometryFix.isPresent()) {
				odometryPoses.add(odometryFix.get());
				PoseEstimatorLogging
					.logStandardDeviations(logPath + "odometry/", PoseEstimationMath.calculateStandardDeviationOfPose(odometryPoses));

				Transform2d sampleDifferenceFromPose = new Transform2d(odometryFix.get(), odometryPose);
				Pose2d fixedPose = observation.getEstimatedPose().toPose2d().plus(sampleDifferenceFromPose);
				Logger.recordOutput(logPath + "fixedObservation/" + observationCount, fixedPose);
				output.add(fixedPose);
			}
			;
		}
		return output;
	}

	private Pose2d getFilterResult(double x, double y, double ang) {
		return new Pose2d(x, y, Rotation2d.fromRadians(ang));
	}

	public Optional<ProcessedVisionData> calculateFixedObservationByOdometryLinearFilter(
		Pose2d odometryPose,
		TimeInterpolatableBuffer<Pose2d> odometryPoseInterpolator
	) {
		if (observations.size() != maximumSize) {
			return Optional.empty();
		}

		ProcessedVisionData calculatedFixedPoseByOdometryLinearFilter = calculateFixedPoseByOdometryLinearFilter(
			odometryPose,
			odometryPoseInterpolator
		);
		Logger.recordOutput(logPath + "stdDevs", calculatedFixedPoseByOdometryLinearFilter.getStdDev());
		return Optional.of(
			new ProcessedVisionData(
				calculatedFixedPoseByOdometryLinearFilter.getEstimatedPose(),
				TimeUtils.getCurrentTimeSeconds(),
				calculatedFixedPoseByOdometryLinearFilter.getStdDev()
			)
		);
	}

	private ProcessedVisionData calculateFixedPoseByOdometryLinearFilter(
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

		return new ProcessedVisionData(
			new Pose2d(
				xFilterWithOdometryFix.calculate(lastObservationPose.getX()),
				yFilterWithOdometryFix.calculate(lastObservationPose.getY()),
				Rotation2d.fromRadians(angleFilterRadiansWithOdometryFix.calculate(lastObservationPose.getRotation().getRadians()))
			),
			TimeUtils.getCurrentTimeSeconds(),
			PoseEstimationMath.calculateStandardDeviationOfPose(fixedData)
		);
	}

	public Optional<ProcessedVisionData> calculateLinearFilterResult() {
		if (observations.isEmpty()) {
			return Optional.empty();
		}
		double timestamp = getLastObservation(observations).getTimestamp();
		return Optional.of(
			new ProcessedVisionData(
				lastFilterOutput,
				timestamp,
				PoseEstimationMath.calculateStandardDeviationOfPose(getRobotPosesFromGivenObservations())
			)
		);
	}

}
