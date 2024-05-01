package frc.robot.subsystems.swerve.gyro.gyropigeon2;

import com.ctre.phoenix6.BaseStatusSignal;
import frc.utils.devicewrappers.Pigeon2Wrapper;

class GyroPigeon2ConfigObject {

    private final Pigeon2Wrapper gyro;

    private final GyroPigeon2Records.GyroPigeon2Signals signals;

    protected GyroPigeon2ConfigObject(int id, String busChain) {
        this.gyro = new Pigeon2Wrapper(id, busChain);
        this.signals = new GyroPigeon2Records.GyroPigeon2Signals(
                gyro.getYaw(),
                gyro.getPitch(),
                gyro.getAccelerationX(),
                gyro.getAccelerationY(),
                gyro.getAccelerationZ()
        );
        configGyro();
        optimizeBusAndSignalOfGyro();
    }

    private void configGyro() {
        gyro.getConfigurator().apply(GyroPigeon2Constants.PIGEON_2_CONFIGURATION);
    }

    private void optimizeBusAndSignalOfGyro() {
        BaseStatusSignal.setUpdateFrequencyForAll(
                50,
                signals.X_ACCELERATION_SIGNAL(),
                signals.Y_ACCELERATION_SIGNAL(),
                signals.Z_ACCELERATION_SIGNAL()
        );

        signals.PITCH_SIGNAL().setUpdateFrequency(100);
        signals.YAW_SIGNAL().setUpdateFrequency(250); // PoseEstimatorConstants.ODOMETRY_FREQUENCY_HERTZ

        gyro.optimizeBusUtilization();
    }

    protected Pigeon2Wrapper getGyro() {
        return gyro;
    }

    protected GyroPigeon2Records.GyroPigeon2Signals getSignals() {
        return signals;
    }

}
