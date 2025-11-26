package frc.robot.subsystems.arm;

import edu.wpi.first.math.geometry.Rotation2d;

public record ArmSimulationConstants(
        Rotation2d maxPosition,
        Rotation2d minPosition,
        Rotation2d startingPosition,
        double momentOfInertia,
        double armLengthMeters
) {
}
