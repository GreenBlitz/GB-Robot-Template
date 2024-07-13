package frc.robot.subsystems.swerve.gyro.pigeon2;

import com.ctre.phoenix6.StatusSignal;

public record Pigeon2GyroSignals(
            StatusSignal<Double> yawSignal,
            StatusSignal<Double> xAccelerationSignal,
            StatusSignal<Double> yAccelerationSignal,
            StatusSignal<Double> zAccelerationSignal
) {}
