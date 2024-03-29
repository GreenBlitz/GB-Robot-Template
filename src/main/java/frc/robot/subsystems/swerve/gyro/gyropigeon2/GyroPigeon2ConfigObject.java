package frc.robot.subsystems.swerve.gyro.gyropigeon2;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.Pigeon2;

public class GyroPigeon2ConfigObject {

    private Pigeon2 gyro;

    public StatusSignal<Double> YAW_SIGNAL, PITCH_SIGNAL, X_ACCELERATION_SIGNAL, Y_ACCELERATION_SIGNAL, Z_ACCELERATION_SIGNAL;

    protected GyroPigeon2ConfigObject(int id, String busChain){
        this.gyro = new Pigeon2(id, busChain);

        configGyro();
        optimizeBusAndSignalOfGyro();
    }

    private void configGyro(){
        gyro.getConfigurator().apply(GyroPigeon2Constants.PIGEON_2_CONFIGURATION);
    }

    private void optimizeBusAndSignalOfGyro() {
        YAW_SIGNAL = gyro.getYaw();
        PITCH_SIGNAL = gyro.getPitch();
        X_ACCELERATION_SIGNAL = gyro.getAccelerationX();
        Y_ACCELERATION_SIGNAL = gyro.getAccelerationY();
        Z_ACCELERATION_SIGNAL = gyro.getAccelerationZ();

        BaseStatusSignal.setUpdateFrequencyForAll(
                50,
                X_ACCELERATION_SIGNAL,
                Y_ACCELERATION_SIGNAL,
                Z_ACCELERATION_SIGNAL
        );

        PITCH_SIGNAL.setUpdateFrequency(100);
        YAW_SIGNAL.setUpdateFrequency(250);//PoseEstimatorConstants.ODOMETRY_FREQUENCY_HERTZ

        gyro.optimizeBusUtilization();
    }

    public Pigeon2 getGyro() {
        return gyro;
    }
}
