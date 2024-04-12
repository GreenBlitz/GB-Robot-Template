package frc.robot.subsystems.swerve.gyro.gyropigeon2;

import com.ctre.phoenix6.StatusSignal;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.constants.Phoenix6Constants;
import frc.robot.constants.Ports;

public class GyroPigeon2 {

    private final GyroPigeon2Status gyroPigeon2Data;
    private final GyroPigeon2Actions gyroPigeon2Actions;

    public GyroPigeon2() {
        GyroPigeon2ConfigObject gyroPigeon2ConfigObject = new GyroPigeon2ConfigObject(Ports.PIGEON_2_ID,
                Phoenix6Constants.CANIVORE_NAME
        );
        this.gyroPigeon2Data = new GyroPigeon2Status(gyroPigeon2ConfigObject.getSignals());
        this.gyroPigeon2Actions = new GyroPigeon2Actions(gyroPigeon2ConfigObject.getGyro());
    }

    public void setYaw(Rotation2d angle) {
        gyroPigeon2Actions.setYaw(angle);
    }

    public StatusSignal<Double> getYAW_SIGNAL() {
        return gyroPigeon2Data.getYAW_SIGNAL();
    }

    public StatusSignal<Double> getPITCH_SIGNAL() {
        return gyroPigeon2Data.getPITCH_SIGNAL();
    }

    public StatusSignal<Double> getX_ACCELERATION_SIGNAL() {
        return gyroPigeon2Data.getX_ACCELERATION_SIGNAL();
    }

    public StatusSignal<Double> getY_ACCELERATION_SIGNAL() {
        return gyroPigeon2Data.getY_ACCELERATION_SIGNAL();
    }

    public StatusSignal<Double> getZ_ACCELERATION_SIGNAL() {
        return gyroPigeon2Data.getZ_ACCELERATION_SIGNAL();
    }
}
