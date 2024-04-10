package frc.robot.subsystems.swerve.gyro.gyropigeon2;

import com.ctre.phoenix6.StatusSignal;

record GyroPigeon2Signals(
        StatusSignal<Double> YAW_SIGNAL,
        StatusSignal<Double> PITCH_SIGNAL,
        StatusSignal<Double> X_ACCELERATION_SIGNAL,
        StatusSignal<Double> Y_ACCELERATION_SIGNAL,
        StatusSignal<Double> Z_ACCELERATION_SIGNAL) {}
