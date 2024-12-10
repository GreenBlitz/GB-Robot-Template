package frc.robot.vision.sources.simulationsource;


import edu.wpi.first.math.geometry.Rotation2d;

public record SimulatedSourceConfiguration(
	double angleNoiseScaling,
	double transformNoiseScaling,
	double spikesProbability,
	double maximumSpikeMeters,
	double detectionRangeMeters,
	Rotation2d fieldOfView
) {}

