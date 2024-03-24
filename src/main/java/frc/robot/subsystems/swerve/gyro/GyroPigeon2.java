package frc.robot.subsystems.swerve.gyro;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.Pigeon2;
import frc.robot.constants.Phoenix6Constants;
import frc.robot.constants.Ports;
import frc.robot.subsystems.swerve.mk4iswerve.MK4ISwerveConstants;

public class GyroPigeon2 {

    private static GyroPigeon2 instance;

    private final Pigeon2 gyro;
    public StatusSignal<Double> YAW_SIGNAL, PITCH_SIGNAL, X_ACCELERATION_SIGNAL, Y_ACCELERATION_SIGNAL, Z_ACCELERATION_SIGNAL;

    public GyroPigeon2(){
        this.gyro = new Pigeon2(Ports.PIGEON_2_ID, Phoenix6Constants.CANIVORE_NAME);
        configGyro();
        optimizeBusAndSignalOfGyro();
    }

    public static GyroPigeon2 getInstance() {
        if (instance == null){
            instance = new GyroPigeon2();
        }
        return instance;
    }

    public Pigeon2 getGyro() {
        return gyro;
    }

    public void configGyro(){
        gyro.getConfigurator().apply(MK4ISwerveConstants.PIGEON_2_CONFIGURATION);
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
}
