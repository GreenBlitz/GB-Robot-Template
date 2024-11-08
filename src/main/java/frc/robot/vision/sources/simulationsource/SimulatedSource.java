package frc.robot.vision.sources.simulationsource;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.geometry.*;
import frc.robot.poseestimator.PoseEstimationMath;
import frc.robot.poseestimator.PoseEstimatorConstants;
import frc.robot.subsystems.GBSubsystem;
import frc.robot.vision.GyroAngleValues;
import frc.robot.vision.RawVisionData;
import frc.robot.vision.sources.VisionSource;
import frc.utils.time.TimeUtils;
import org.littletonrobotics.junction.Logger;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.Random;
import java.util.function.Supplier;

public class SimulatedSource extends GBSubsystem implements VisionSource<RawVisionData> {

	private final Random randomValuesGenerator;
	private final Rotation2d fieldOfView;

	private final double detectionRangeMeters;
	private final double spikesProbability;
	private final double maximumSpikeMeters;
	private final List<RawVisionData> currentObservations;

	private final Supplier<Pose2d> simulateRobotPose;
	private final Supplier<Double> transformNoise;
	private final Supplier<Double> angleNoise;

	public SimulatedSource(String cameraName, Supplier<Pose2d> simulateRobotPose, SimulatedSourceConfiguration config) {
		super(cameraName + "Simulated/");

		this.randomValuesGenerator = new Random();
		this.currentObservations = new ArrayList<>();

		this.detectionRangeMeters = config.detectionRangeMeters();
		this.spikesProbability = config.spikesProbability();
		this.maximumSpikeMeters = config.maximumSpikeMeters();
		this.fieldOfView = config.fieldOfView();

		this.simulateRobotPose = simulateRobotPose;
		this.angleNoise = () -> randomValuesGenerator.nextGaussian() * config.angleNoiseScaling();
		this.transformNoise = () -> randomValuesGenerator.nextGaussian() * config.transformNoiseScaling();
	}

	@Override
	public void updateGyroAngles(GyroAngleValues gyroAngleValues) {}

	@Override
	public void update() {
		Pose2d simulatedPose = simulateRobotPose.get();
		currentObservations.clear();

		for (AprilTag aprilTag : PoseEstimatorConstants.APRIL_TAG_FIELD.loadAprilTagLayoutField().getTags()) {
			Pose3d aprilTagPose = aprilTag.pose;
			String logPath = super.getLogPath() + "IDs/" + aprilTag.ID;

			if (PoseEstimationMath.distanceBetweenPosesMeters(aprilTagPose.toPose2d(), simulatedPose) <= detectionRangeMeters) {
				if (isRobotPointingIntoAngle(aprilTagPose.getRotation().toRotation2d())) {
					Pose2d noisedPose = calculateNoisedPose();
					currentObservations.add(constructRawVisionData(noisedPose, aprilTagPose));
					Logger.recordOutput(logPath, "returning");
				} else {
					Logger.recordOutput(logPath, "not facing");
				}
			} else {
				Logger.recordOutput(logPath, "far away");
			}
		}
	}

	public RawVisionData constructRawVisionData(Pose2d noisedPose, Pose3d aprilTagPose) {
		return new RawVisionData(
		//@formatter:off
			new Pose3d(
				new Translation3d(noisedPose.getX(), noisedPose.getY(), 0),
				new Rotation3d(0, 0, noisedPose.getRotation().getRadians())
			),
			//@formatter:on
			aprilTagPose.getZ(),
			PoseEstimationMath.distanceBetweenPosesMeters(aprilTagPose.toPose2d(), calculateNoisedPose()),
			TimeUtils.getCurrentTimeSeconds()
		);
	}

	public boolean isRobotPointingIntoAngle(Rotation2d angle) {
		Rotation2d robotAngle = simulateRobotPose.get().getRotation();
		double angleDeltaRadians = Math.abs(Math.PI - robotAngle.minus(angle).getRadians());
		return (angleDeltaRadians) < (fieldOfView.getRadians() / 2);
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
		}
		RawVisionData output = currentObservations.get(0);
		Logger.recordOutput(super.getLogPath() + "position", output.estimatedPose());
		return Optional.of(output);
	}

	@Override
	public Optional<Rotation2d> getRobotHeading() {
		return Optional.empty();
	}

	@Override
	public void updateCurrentEstimatedPose(Pose2d estimatedPose) {}

}
