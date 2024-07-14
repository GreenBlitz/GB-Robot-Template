package frc.robot.subsystems.swerve.modules.mk4i;

import com.ctre.phoenix6.StatusSignal;
import frc.utils.devicewrappers.TalonFXWrapper;

class MK4IModuleRecords {

    public record MK4IModuleMotors(TalonFXWrapper driveMotor, TalonFXWrapper steerMotor) {}

    public record MK4IModuleSignals(
            StatusSignal<Double> encoderPositionSignal,
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
