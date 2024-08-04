package frc.robot.subsystems.swerve.modules.drive.talonfx;

import com.ctre.phoenix6.StatusSignal;

record TalonFXDriveSignals(
        StatusSignal<Double> positionSignal,
        StatusSignal<Double> velocitySignal,
        StatusSignal<Double> accelerationSignal,
        StatusSignal<Double> voltageSignal,
        StatusSignal<Double> statorCurrentSignal
) {}
