package frc.robot.subsystems.swerve.gyro.gyropigeon2;

import com.ctre.phoenix6.StatusSignal;

public class GyroPigeon2Data {

    private final StatusSignal<Double> YAW_SIGNAL, PITCH_SIGNAL, X_ACCELERATION_SIGNAL, Y_ACCELERATION_SIGNAL, Z_ACCELERATION_SIGNAL;

    protected GyroPigeon2Data(GyroPigeon2ConfigObject gyroPigeon2ConfigObject){
        this.YAW_SIGNAL = gyroPigeon2ConfigObject.YAW_SIGNAL;
        this.PITCH_SIGNAL = gyroPigeon2ConfigObject.PITCH_SIGNAL;
        this.X_ACCELERATION_SIGNAL = gyroPigeon2ConfigObject.X_ACCELERATION_SIGNAL;
        this.Y_ACCELERATION_SIGNAL = gyroPigeon2ConfigObject.Y_ACCELERATION_SIGNAL;
        this.Z_ACCELERATION_SIGNAL = gyroPigeon2ConfigObject.Z_ACCELERATION_SIGNAL;
    }

    public StatusSignal<Double> getYAW_SIGNAL() {
        return YAW_SIGNAL.refresh();
    }

    public StatusSignal<Double> getPITCH_SIGNAL() {
        return PITCH_SIGNAL.refresh();
    }

    public StatusSignal<Double> getX_ACCELERATION_SIGNAL() {
        return X_ACCELERATION_SIGNAL.refresh();
    }

    public StatusSignal<Double> getY_ACCELERATION_SIGNAL() {
        return Y_ACCELERATION_SIGNAL.refresh();
    }

    public StatusSignal<Double> getZ_ACCELERATION_SIGNAL() {
        return Z_ACCELERATION_SIGNAL.refresh();
    }

}
