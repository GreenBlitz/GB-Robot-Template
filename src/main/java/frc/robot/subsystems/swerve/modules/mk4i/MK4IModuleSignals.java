package frc.robot.subsystems.swerve.modules.mk4i;

import com.ctre.phoenix6.StatusSignal;

public record MK4IModuleSignals(
        StatusSignal<Double> encoderAbsolutePositionSignal,
        StatusSignal<Double> encoderVelocitySignal,
        StatusSignal<Double> encoderVoltageSignal,

        StatusSignal<Double> steerMotorPositionSignal,
        StatusSignal<Double> steerMotorVelocitySignal,
        StatusSignal<Double> steerMotorAccelerationSignal,
        StatusSignal<Double> steerMotorVoltageSignal,

        StatusSignal<Double> drivePositionSignal,
        StatusSignal<Double> driveVelocitySignal,
        StatusSignal<Double> driveAccelerationSignal,
        StatusSignal<Double> driveVoltageSignal,
        StatusSignal<Double> driveStatorCurrentSignal
) {}
