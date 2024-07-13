package frc.robot.subsystems.swerve.gyro.pigeon2;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.utils.ctre.PhoenixProUtils;

class Pigeon2GyroStatus {

    private final Pigeon2GyroSignals pigeon2GyroSignals;

    protected Pigeon2GyroStatus(Pigeon2GyroSignals pigeon2GyroSignals) {
        this.pigeon2GyroSignals = pigeon2GyroSignals;
    }

    public StatusCode refreshAllSignals() {
        return BaseStatusSignal.refreshAll(
                getYawSignal(false),
                getXAccelerationSignal(false),
                getYAccelerationSignal(false),
                getZAccelerationSignal(false)
        );
    }


    public Rotation2d getYaw(boolean refresh) {
        return Rotation2d.fromDegrees(getYawSignal(refresh).getValue());
    }

    public StatusSignal<Double> getYawSignal(boolean refresh) {
        return PhoenixProUtils.getRefreshedSignal(refresh, pigeon2GyroSignals.yawSignal());
    }


    public StatusSignal<Double> getXAccelerationSignal(boolean refresh) {
        return PhoenixProUtils.getRefreshedSignal(refresh, pigeon2GyroSignals.xAccelerationSignal());
    }

    public StatusSignal<Double> getYAccelerationSignal(boolean refresh) {
        return PhoenixProUtils.getRefreshedSignal(refresh, pigeon2GyroSignals.yAccelerationSignal());
    }

    public StatusSignal<Double> getZAccelerationSignal(boolean refresh) {
        return PhoenixProUtils.getRefreshedSignal(refresh, pigeon2GyroSignals.zAccelerationSignal());
    }

}
