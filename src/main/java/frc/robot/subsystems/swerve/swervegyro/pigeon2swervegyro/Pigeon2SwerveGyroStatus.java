package frc.robot.subsystems.swerve.swervegyro.pigeon2swervegyro;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import edu.wpi.first.math.geometry.Rotation2d;

class Pigeon2SwerveGyroStatus {

    private final Pigeon2SwerveGyroRecords.GyroPigeon2Signals gyroPigeon2Signals;

    protected Pigeon2SwerveGyroStatus(Pigeon2SwerveGyroRecords.GyroPigeon2Signals gyroPigeon2Signals) {
        this.gyroPigeon2Signals = gyroPigeon2Signals;
    }


    public Rotation2d getYaw(boolean refresh) {
        return Rotation2d.fromDegrees(getYAW_SIGNAL(refresh).getValue());
    }

    public Rotation2d getPitch(boolean refresh) {
        return Rotation2d.fromDegrees(getPITCH_SIGNAL(refresh).getValue());
    }

    public StatusSignal<Double> getYAW_SIGNAL(boolean refresh) {
        return getSignal(refresh, gyroPigeon2Signals.YAW_SIGNAL());
    }

    public StatusSignal<Double> getPITCH_SIGNAL(boolean refresh) {
        return getSignal(refresh, gyroPigeon2Signals.PITCH_SIGNAL());
    }

    public StatusSignal<Double> getX_ACCELERATION_SIGNAL(boolean refresh) {
        return getSignal(refresh, gyroPigeon2Signals.X_ACCELERATION_SIGNAL());
    }

    public StatusSignal<Double> getY_ACCELERATION_SIGNAL(boolean refresh) {
        return getSignal(refresh, gyroPigeon2Signals.Y_ACCELERATION_SIGNAL());
    }

    public StatusSignal<Double> getZ_ACCELERATION_SIGNAL(boolean refresh) {
        return getSignal(refresh, gyroPigeon2Signals.Z_ACCELERATION_SIGNAL());
    }

    private <T> StatusSignal<T> getSignal(boolean refresh, StatusSignal<T> signal) {
        return refresh ? signal.refresh() : signal;
    }

    public void refreshAllSignals() {
        BaseStatusSignal.refreshAll(
                getYAW_SIGNAL(false),
                getPITCH_SIGNAL(false),
                getX_ACCELERATION_SIGNAL(false),
                getY_ACCELERATION_SIGNAL(false),
                getZ_ACCELERATION_SIGNAL(false)
        );
    }

}
