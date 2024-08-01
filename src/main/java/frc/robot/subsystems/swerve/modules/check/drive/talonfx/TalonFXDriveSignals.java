package frc.robot.subsystems.swerve.modules.check.drive.talonfx;

import com.ctre.phoenix6.StatusSignal;

public record TalonFXDriveSignals(
        StatusSignal<Double> drivePositionSignal,
        StatusSignal<Double> driveVelocitySignal,
        StatusSignal<Double> driveAccelerationSignal,
        StatusSignal<Double> driveVoltageSignal,
        StatusSignal<Double> driveStatorCurrentSignal
) {}
