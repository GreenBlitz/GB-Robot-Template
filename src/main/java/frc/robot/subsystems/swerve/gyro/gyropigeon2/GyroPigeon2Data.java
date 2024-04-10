package frc.robot.subsystems.swerve.gyro.gyropigeon2;

import com.ctre.phoenix6.StatusSignal;

class GyroPigeon2Data {

    private final GyroPigeon2Records.GyroPigeon2Signals gyroPigeon2Signals;

    protected GyroPigeon2Data(GyroPigeon2Records.GyroPigeon2Signals gyroPigeon2Signals) {
        this.gyroPigeon2Signals = gyroPigeon2Signals;
    }

    public StatusSignal<Double> getYAW_SIGNAL() {
        return gyroPigeon2Signals.YAW_SIGNAL().refresh();
    }

    public StatusSignal<Double> getPITCH_SIGNAL() {
        return gyroPigeon2Signals.PITCH_SIGNAL().refresh();
    }

    public StatusSignal<Double> getX_ACCELERATION_SIGNAL() {
        return gyroPigeon2Signals.X_ACCELERATION_SIGNAL().refresh();
    }

    public StatusSignal<Double> getY_ACCELERATION_SIGNAL() {
        return gyroPigeon2Signals.Y_ACCELERATION_SIGNAL().refresh();
    }

    public StatusSignal<Double> getZ_ACCELERATION_SIGNAL() {
        return gyroPigeon2Signals.Z_ACCELERATION_SIGNAL().refresh();
    }
}
