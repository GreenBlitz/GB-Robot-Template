package frc.robot.subsystems.swerve.module;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.hardware.interfaces.InputSignal;

public record ModuleSignals(
	InputSignal<Rotation2d> driveVelocitySignal,
	InputSignal<Double> driveVoltageSignal,
	InputSignal<Double> driveCurrentSignal,
	InputSignal<Rotation2d> steerAngleSignal,
	InputSignal<Double> steerVoltageSignal
) {}
