package frc.robot.vision.sources.simulationsource;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.geometry.*;
import frc.robot.subsystems.GBSubsystem;
import frc.robot.vision.VisionConstants;
import frc.robot.vision.filters.Filter;
import frc.robot.vision.rawdata.RawAprilTagVisionData;
import frc.robot.vision.sources.VisionSource;
import frc.utils.time.TimeUtils;
import org.littletonrobotics.junction.Logger;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.Random;
import java.util.function.Supplier;

public class SimulatedSource extends GBSubsystem implements VisionSource<RawAprilTagVisionData> {

	private final Random randomValuesGenerator;
	private final Rotation2d fieldOfView;

	private final double detectionRangeMeters;
	private final double spikesProbability;
	private final double maximumSpikeMeters;
	private final List<RawAprilTagVisionData> currentObservations;

	private final Supplier<Pose3d> cameraPose;
	private final Supplier<Pose2d> simulateRobotPose;
	private final Supplier<Double> transformNoise;
	private final Supplier<Double> angleNoise;

	private Filter<RawAprilTagVisionData> filter;

	public SimulatedSource(
		String cameraName,
		Supplier<Pose2d> simulateRobotPose,
		Pose3d camerasRelativeToRobot,
		SimulatedSourceConfiguration config,
		Filter<RawAprilTagVisionData> filter
	) {
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

		this.cameraPose = () -> new Pose3d(simulateRobotPose.get()).relativeTo(camerasRelativeToRobot);
		this.filter = filter;
	}

	@Override
	public void update() {
		Pose2d simulatedPose = simulateRobotPose.get();
		currentObservations.clear();

		for (AprilTag aprilTag : VisionConstants.APRIL_TAG_FIELD_LAYOUT.getTags()) {
			Pose3d aprilTagPose = aprilTag.pose;
			String logPath = super.getLogPath() + "IDs/" + aprilTag.ID;

			double distanceMeters = distanceBetweenPosesMeters(aprilTagPose, cameraPose.get());
			if (distanceMeters <= detectionRangeMeters) {
				if (isRobotPointingIntoAngle(aprilTagPose.getRotation().toRotation2d())) {
					Pose2d noisedPose = calculateNoisedPose();
					RawAprilTagVisionData visionInput = constructRawVisionData(noisedPose, aprilTagPose, aprilTag);
					currentObservations.add(visionInput);
					Logger.recordOutput(logPath + "state", "returning");
					Logger.recordOutput(logPath + "latestOutputPose", visionInput.getEstimatedPose());
				} else {
					Logger.recordOutput(logPath + "state", "not facing");
				}
			} else {
				Logger.recordOutput(logPath + "state", "d=" + distanceMeters + "m");
			}
		}
	}

	public RawAprilTagVisionData constructRawVisionData(Pose2d noisedPose, Pose3d aprilTagPose, AprilTag aprilTag) {
		return new RawAprilTagVisionData(
			new Pose3d(new Translation3d(noisedPose.getX(), noisedPose.getY(), 0), new Rotation3d(0, 0, noisedPose.getRotation().getRadians())),
			aprilTagPose.getZ(),
			distanceBetweenPosesMeters(aprilTagPose.toPose2d(), calculateNoisedPose()),
			TimeUtils.getCurrentTimeSeconds(),
			aprilTag
		);
	}

	public boolean isRobotPointingIntoAngle(Rotation2d angle) {
		Rotation2d cameraAngle = cameraPose.get().getRotation().toRotation2d();
		double angleDeltaRadians = Math.abs(cameraAngle.minus(angle).getRadians());
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

	public Optional<RawAprilTagVisionData> getLatestObservation() {
		if (currentObservations.isEmpty()) {
			return Optional.empty();
		}
		var output = currentObservations.get(0);
		return Optional.of(output);
	}

	public Optional<RawAprilTagVisionData> getRawVisionEstimation() {
		return getLatestObservation();
	}

	@Override
	public boolean shouldDataBeFiltered() {
		return getLatestObservation().map(filter::doesFilterPasses).orElseGet(() -> true);
	}

	@Override
	public Filter<RawAprilTagVisionData> setFilter(Filter<RawAprilTagVisionData> newFilter) {
		return this.filter = newFilter;
	}

	private void logMovingData() {
		getLatestObservation().ifPresent(
			(RawAprilTagVisionData rawVisionData) -> Logger.recordOutput(super.getLogPath() + "position", rawVisionData.getEstimatedPose())
		);
		Logger.recordOutput(getLogPath() + "cameraPose", cameraPose.get());
	}

	@Override
	protected void subsystemPeriodic() {
		logMovingData();
	}

	// Would be replaced by PoseEstimationMath.distanceBetweenPosesMeters once it'll get merged
	private double distanceBetweenPosesMeters(Pose3d pose1, Pose3d pose2) {
		return pose1.minus(pose2).getTranslation().getNorm();
	}

	private double distanceBetweenPosesMeters(Pose2d pose1, Pose2d pose2) {
		return pose1.minus(pose2).getTranslation().getNorm();
	}

}
