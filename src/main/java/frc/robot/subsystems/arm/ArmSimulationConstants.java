package frc.robot.subsystems.arm;

import org.wpilib.math.geometry.Angle2d;

public record ArmSimulationConstants(
	Angle2d maxPosition,
	Angle2d minPosition,
	Angle2d startingPosition,
	double momentOfInertiaKgMeterSquared,
	double armLengthMeters
) {}
