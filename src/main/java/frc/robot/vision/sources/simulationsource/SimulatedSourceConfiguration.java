package frc.robot.vision.sources.simulationsource;


public record SimulatedSourceConfiguration(
	double angleNoiseScaling,
	double transformNoiseScaling,
	double spikesProbability,
	double maximumSpikeMeters,
	double detectionRangeMeters
) {}
