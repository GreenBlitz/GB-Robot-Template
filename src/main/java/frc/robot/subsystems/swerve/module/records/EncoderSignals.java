package frc.robot.subsystems.swerve.module.records;

import org.wpilib.math.geometry.Rotation2d;
import frc.robot.hardware.interfaces.InputSignal;

public record EncoderSignals(InputSignal<Rotation2d> position) {}
