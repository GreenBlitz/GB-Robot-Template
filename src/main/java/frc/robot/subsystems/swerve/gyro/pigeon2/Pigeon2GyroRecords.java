package frc.robot.subsystems.swerve.gyro.pigeon2;

import com.ctre.phoenix6.StatusSignal;

class Pigeon2GyroRecords {

    protected record GyroPigeon2Signals(
            StatusSignal<Double> yawSignal,
            StatusSignal<Double> xAccelerationSignal,
            StatusSignal<Double> yAccelerationSignal,
            StatusSignal<Double> zAccelerationSignal
    ) {}

}
