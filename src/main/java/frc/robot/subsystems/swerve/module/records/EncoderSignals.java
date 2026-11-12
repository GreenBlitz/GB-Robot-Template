package frc.robot.subsystems.swerve.module.records;

import frc.utils.angle2d.Angle2d;
import frc.robot.hardware.interfaces.InputSignal;

public record EncoderSignals(InputSignal<Angle2d> position) {}
