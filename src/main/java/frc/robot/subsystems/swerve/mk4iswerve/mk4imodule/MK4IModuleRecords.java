package frc.robot.subsystems.swerve.mk4iswerve.mk4imodule;

import com.ctre.phoenix6.StatusSignal;
import frc.utils.devicewrappers.TalonFXWrapper;

class MK4IModuleRecords {

    public record MK4IModuleMotors(TalonFXWrapper driveMotor, TalonFXWrapper steerMotor) {}

    public record MK4IModuleSignals(
            StatusSignal<Double> steerEncoderAbsolutePositionSignal, StatusSignal<Double> steerEncoderVelocitySignal,
            StatusSignal<Double> steerEncoderVoltageSignal, StatusSignal<Double> drivePositionSignal,
            StatusSignal<Double> driveVelocitySignal, StatusSignal<Double> driveAccelerationSignal,
            StatusSignal<Double> driveVoltageSignal, StatusSignal<Double> driveStatorCurrentSignal,
            StatusSignal<Double> steerPositionSignal, StatusSignal<Double> steerVelocitySignal,
            StatusSignal<Double> steerAccelerationSignal, StatusSignal<Double> steerVoltageSignal
    ) {}

}
