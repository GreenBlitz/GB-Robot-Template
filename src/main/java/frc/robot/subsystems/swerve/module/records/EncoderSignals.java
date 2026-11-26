package frc.robot.subsystems.swerve.module.records;

import frc.robot.hardware.interfaces.InputSignal;
import org.wpilib.math.geometry.Rotation2d;

public record EncoderSignals(InputSignal<Rotation2d> position) {}
