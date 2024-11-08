package frc.robot.vision.sources.simulationsource;

import edu.wpi.first.math.geometry.Pose2d;

import java.util.function.Supplier;

public record SimulatedSourceConfiguration(
	double angleNoiseScaling,
	double transformNoiseScaling,
	double spikesProbability,
	double maximumSpikeMeters,
	double detectionRangeMeters
) {}
