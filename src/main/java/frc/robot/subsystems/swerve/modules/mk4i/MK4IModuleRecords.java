package frc.robot.subsystems.swerve.modules.mk4i;

import com.ctre.phoenix6.StatusSignal;

class MK4IModuleRecords {

    public record MK4IModuleSignals(
            StatusSignal<Double> encoderAbsolutePositionSignal,
            StatusSignal<Double> encoderVelocitySignal,
            StatusSignal<Double> encoderVoltageSignal,

            StatusSignal<Double> drivePositionSignal,
            StatusSignal<Double> driveVelocitySignal,
            StatusSignal<Double> driveAccelerationSignal,
            StatusSignal<Double> driveVoltageSignal,
            StatusSignal<Double> driveStatorCurrentSignal,

            StatusSignal<Double> steerMotorPositionSignal,
            StatusSignal<Double> steerMotorVelocitySignal,
            StatusSignal<Double> steerMotorAccelerationSignal,
            StatusSignal<Double> steerMotorVoltageSignal
    ) {}

}
