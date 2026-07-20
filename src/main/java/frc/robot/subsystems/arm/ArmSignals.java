package frc.robot.subsystems.arm;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.hardware.interfaces.InputSignal;

public record ArmSignals(
	InputSignal<Double> voltage,
	InputSignal<Double> current,
	InputSignal<Rotation2d> velocity,
	InputSignal<Rotation2d> position
) {}
