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

	private final Supplier<Pose3d> cameraPose;
	private final Supplier<Pose2d> simulateRobotPose;
	private final Supplier<Double> transformNoise;
	private final Supplier<Double> angleNoise;

	public SimulatedSource(
		String cameraName,
		Supplier<Pose2d> simulateRobotPose,
		Pose3d camerasRelativeToRobot,
		SimulatedSourceConfiguration config
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

			double distanceMeters = PoseEstimationMath.distanceBetweenPosesMeters(aprilTagPose, cameraPose.get());
			if (distanceMeters <= detectionRangeMeters) {
				if (isRobotPointingIntoAngle(aprilTagPose.getRotation().toRotation2d())) {
					Pose2d noisedPose = calculateNoisedPose();
					RawVisionData visionInput = constructRawVisionData(noisedPose, aprilTagPose);
					currentObservations.add(visionInput);
					Logger.recordOutput(logPath + "state", "returning");
					Logger.recordOutput(logPath + "latestOutputPose", visionInput.estimatedPose());
				} else {
					Logger.recordOutput(logPath + "state", "not facing");
				}
			} else {
				Logger.recordOutput(logPath + "state", "d=" + distanceMeters + "m");
			}
		}
	}

	public RawVisionData constructRawVisionData(Pose2d noisedPose, Pose3d aprilTagPose) {
		return new RawVisionData(
			new Pose3d(new Translation3d(noisedPose.getX(), noisedPose.getY(), 0), new Rotation3d(0, 0, noisedPose.getRotation().getRadians())),
			aprilTagPose.getZ(),
			PoseEstimationMath.distanceBetweenPosesMeters(aprilTagPose.toPose2d(), calculateNoisedPose()),
			TimeUtils.getCurrentTimeSeconds()
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

	public Optional<RawVisionData> getLatestObservation() {
		if (currentObservations.isEmpty()) {
			return Optional.empty();
		}
		var output = currentObservations.get(0);
		return Optional.of(output);
	}

	@Override
	public Optional<RawVisionData> getAllData() {
		return getLatestObservation();
	}

	@Override
	public Optional<Rotation2d> getRobotHeading() {
		Optional<Rotation2d> heading = getLatestObservation().map(rawVisionData -> rawVisionData.estimatedPose().toPose2d().getRotation());
		heading.ifPresent((Rotation2d robotHeading) -> Logger.recordOutput(getLogPath() + "heading", robotHeading));
		return heading;
	}

	@Override
	public void updateCurrentEstimatedPose(Pose2d estimatedPose) {}

	private void logMovingData() {
		getLatestObservation()
			.ifPresent((RawVisionData rawVisionData) -> Logger.recordOutput(super.getLogPath() + "position", rawVisionData.estimatedPose()));
		Logger.recordOutput(getLogPath() + "cameraPose", cameraPose.get());
	}

	@Override
	protected void subsystemPeriodic() {
		logMovingData();
	}

}
