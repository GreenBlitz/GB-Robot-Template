package frc.robot.subsystems.swerve.module.records;

import frc.robot.hardware.interfaces.InputSignal;
import frc.utils.costomUnits.Angle2d;

public record EncoderSignals(InputSignal<Angle2d> position) {}
