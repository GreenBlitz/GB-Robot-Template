package frc.robot.vision.sources;

import edu.wpi.first.math.geometry.Pose2d;

import java.util.function.Supplier;

public record SimulatedSourceConfiguration(
	String cameraName,
	String logPath,
	Supplier<Pose2d> robotSimulatedPose,
	double angleNoiseScaling,
	double transformNoiseScaling,
	double spikesProbability,
	double maximumSpikeMeters,
	double detectionRangeMeters
) {

	public SimulatedSourceConfiguration(
		String cameraName,
		Supplier<Pose2d> robotSimulatedPose,
		double angleNoiseScaling,
		double transformNoiseScaling,
		double spikesProbability,
		double maximumSpikeMeters,
		double detectionRangeMeters
	) {
		this(
			cameraName,
			cameraName + "Simulated/",
			robotSimulatedPose,
			angleNoiseScaling,
			transformNoiseScaling,
			spikesProbability,
			maximumSpikeMeters,
			detectionRangeMeters
		);
	}

}
