package frc.robot.subsystems.swerve.module.stuffs;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.hardware.interfaces.IAngleEncoder;
import frc.robot.hardware.interfaces.InputSignal;

public record EncoderStuff(IAngleEncoder encoder, InputSignal<Rotation2d> positionSignal) {}
