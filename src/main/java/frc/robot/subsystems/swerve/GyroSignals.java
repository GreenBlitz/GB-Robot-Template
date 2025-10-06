package frc.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.hardware.interfaces.InputSignal;

public record GyroSignals(InputSignal<Rotation2d> yawSignalThread, InputSignal<Rotation2d> yawSignal) {}
