package frc.robot.subsystems.arm;

import org.wpilib.math.geometry.Rotation2d;

public record ArmSimulationConstants(
	Rotation2d maxPosition,
	Rotation2d minPosition,
	Rotation2d startingPosition,
	double momentOfInertiaKgMeterSquared,
	double armLengthMeters
) {}
