package frc.robot.subsystems.swerve.module.components;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.hardware.angleencoder.IAngleEncoder;
import frc.robot.hardware.signal.InputSignal;

public record EncoderComponents(IAngleEncoder encoder, InputSignal<Rotation2d> positionSignal) {}
