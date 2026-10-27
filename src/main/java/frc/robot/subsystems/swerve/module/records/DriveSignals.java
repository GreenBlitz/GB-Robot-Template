package frc.robot.subsystems.swerve.module.records;

import org.wpilib.math.geometry.Angle2d;
import frc.robot.hardware.interfaces.InputSignal;

public record DriveSignals(
	InputSignal<Angle2d> position,
	InputSignal<Angle2d> velocity,
	InputSignal<Double> current,
	InputSignal<Double> voltage
) {}
