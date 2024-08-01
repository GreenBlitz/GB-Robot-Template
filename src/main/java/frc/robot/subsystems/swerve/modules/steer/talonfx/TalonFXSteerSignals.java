package frc.robot.subsystems.swerve.modules.steer.talonfx;

import com.ctre.phoenix6.StatusSignal;

public record TalonFXSteerSignals(
        StatusSignal<Double> positionSignal,
        StatusSignal<Double> velocitySignal,
        StatusSignal<Double> accelerationSignal,
        StatusSignal<Double> voltageSignal
) {}
