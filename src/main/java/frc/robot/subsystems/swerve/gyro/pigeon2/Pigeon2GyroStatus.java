package frc.robot.subsystems.swerve.gyro.pigeon2;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.utils.ctre.PhoenixProUtils;

class Pigeon2GyroStatus {

    private final Pigeon2GyroRecords.GyroPigeon2Signals gyroPigeon2Signals;

    protected Pigeon2GyroStatus(Pigeon2GyroRecords.GyroPigeon2Signals gyroPigeon2Signals) {
        this.gyroPigeon2Signals = gyroPigeon2Signals;
    }


    public Rotation2d getYaw(boolean refresh) {
        return Rotation2d.fromDegrees(getYawSignal(refresh).getValue());
    }

    public StatusSignal<Double> getYawSignal(boolean refresh) {
        return PhoenixProUtils.getRefreshedSignal(refresh, gyroPigeon2Signals.YAW_SIGNAL());
    }


    public Rotation2d getPitch(boolean refresh) {
        return Rotation2d.fromDegrees(getPitchSignal(refresh).getValue());
    }

    public StatusSignal<Double> getPitchSignal(boolean refresh) {
        return PhoenixProUtils.getRefreshedSignal(refresh, gyroPigeon2Signals.PITCH_SIGNAL());
    }


    public StatusSignal<Double> getXAccelerationSignal(boolean refresh) {
        return PhoenixProUtils.getRefreshedSignal(refresh, gyroPigeon2Signals.X_ACCELERATION_SIGNAL());
    }

    public StatusSignal<Double> getYAccelerationSignal(boolean refresh) {
        return PhoenixProUtils.getRefreshedSignal(refresh, gyroPigeon2Signals.Y_ACCELERATION_SIGNAL());
    }

    public StatusSignal<Double> getZAccelerationSignal(boolean refresh) {
        return PhoenixProUtils.getRefreshedSignal(refresh, gyroPigeon2Signals.Z_ACCELERATION_SIGNAL());
    }


    public StatusCode refreshAllSignals() {
        return BaseStatusSignal.refreshAll(
                getYawSignal(false),
                getPitchSignal(false),
                getXAccelerationSignal(false),
                getYAccelerationSignal(false),
                getZAccelerationSignal(false)
        );
    }

}
