package frc.robot.subsystems.swerve.module.records;

import org.wpilib.math.geometry.Angle2d;
import frc.robot.hardware.interfaces.InputSignal;

public record EncoderSignals(InputSignal<Angle2d> position) {}
