package frc.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.hardware.gyro.IGyro;
import frc.robot.hardware.signal.InputSignal;

public record GyroStuff(String logPath, IGyro gyro, InputSignal<Rotation2d> yawSignal) {}
