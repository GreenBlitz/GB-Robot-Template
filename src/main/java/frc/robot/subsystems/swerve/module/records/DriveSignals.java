package frc.robot.subsystems.swerve.module.records;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.hardware.interfaces.InputSignal;

public record DriveSignals(
	InputSignal<Rotation2d> position,
	InputSignal<Rotation2d> velocity,
	InputSignal<Double> current,
	InputSignal<Double> voltage
) {}
