package frc.robot.vision.sources;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.geometry.*;
import frc.robot.poseestimator.PoseEstimationMath;
import frc.robot.poseestimator.PoseEstimatorConstants;
import frc.robot.subsystems.GBSubsystem;
import frc.robot.vision.GyroAngleValues;
import frc.robot.vision.RawVisionData;
import frc.utils.time.TimeUtils;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.Random;
import java.util.function.Supplier;

public class SimulatedSource extends GBSubsystem implements VisionSource<RawVisionData> {

	private final double detectionRangeMeters;
	private final double spikesProbability;
	private final double maximumSpikeMeters;
	private final Random randomValuesGenerator;
	private final Supplier<Pose2d> simulateRobotPose;
	private final Supplier<Double> transformNoise;
	private final Supplier<Double> angleNoise;
	private final List<RawVisionData> currentObservations;

	public SimulatedSource(SimulatedSourceConfiguration config) {
		super(config.logPath());

		this.detectionRangeMeters = config.detectionRangeMeters();
		this.spikesProbability = config.spikesProbability();
		this.maximumSpikeMeters = config.maximumSpikeMeters();
		this.randomValuesGenerator = new Random();
		this.simulateRobotPose = config.robotSimulatedPose();
		this.angleNoise = () -> randomValuesGenerator.nextGaussian() * config.angleNoiseScaling();
		this.transformNoise = () -> randomValuesGenerator.nextGaussian() * config.transformNoiseScaling();
		this.currentObservations = new ArrayList<>();
	}

	@Override
	public void updateGyroAngles(GyroAngleValues gyroAngleValues) {}

	@Override
	public void update() {
		Pose2d simulatedPose = simulateRobotPose.get();
		currentObservations.clear();

		for (AprilTag aprilTag : PoseEstimatorConstants.aprilTagField.loadAprilTagLayoutField().getTags()) {
			Pose3d aprilTagPose = aprilTag.pose;
			if (PoseEstimationMath.distanceBetweenPosesMeters(aprilTagPose.toPose2d(), simulatedPose) <= detectionRangeMeters) {
				Pose2d noisedPose = calculateNoisedPose();
				currentObservations.add(constructRawVisionData(noisedPose, aprilTagPose));
			}
		}
	}

	public RawVisionData constructRawVisionData(Pose2d noisedPose, Pose3d aprilTagPose) {
		return new RawVisionData(
			new Pose3d(
				new Translation3d(noisedPose.getX(), noisedPose.getY(), 0),
				new Rotation3d(0, 0, noisedPose.getRotation().getRadians())
			),
			aprilTagPose.getZ(),
			PoseEstimationMath.distanceBetweenPosesMeters(aprilTagPose.toPose2d(), calculateNoisedPose()),
			TimeUtils.getCurrentTimeSeconds()
		);
	}

	public Pose2d calculateNoisedPose() {
		Pose2d simulatedPose = simulateRobotPose.get();
		double xSpike = 0;
		double ySpike = 0;
		if (spikesProbability >= randomValuesGenerator.nextDouble()) {
			xSpike = randomValuesGenerator.nextDouble() * maximumSpikeMeters;
			ySpike = randomValuesGenerator.nextDouble() * maximumSpikeMeters;
		}

		return new Pose2d(
			simulatedPose.getX() + transformNoise.get() + xSpike,
			simulatedPose.getY() + transformNoise.get() + ySpike,
			Rotation2d.fromRadians(simulatedPose.getRotation().getRadians() + angleNoise.get())
		);
	}

	@Override
	public Optional<RawVisionData> getAllData() {
		if (currentObservations.isEmpty()) {
			return Optional.empty();
		} else {
			return Optional.of(currentObservations.get(0));
		}
	}

	@Override
	public Optional<Rotation2d> getRobotHeading() {
		return Optional.empty();
	}

	@Override
	public void updateCurrentEstimatedPose(Pose2d estimatedPose) {}

}
