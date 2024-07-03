package frc.robot.subsystems.swerve.gyro.pigeon2gyro;

import com.ctre.phoenix6.BaseStatusSignal;
import frc.robot.poseestimation.poseestimator.PoseEstimatorConstants;
import frc.utils.devicewrappers.Pigeon2Wrapper;

class Pigeon2GyroConfigObject {

    private final Pigeon2Wrapper gyro;

    private final Pigeon2GyroRecords.GyroPigeon2Signals signals;

    protected Pigeon2GyroConfigObject(int id, String busChain) {
        this.gyro = new Pigeon2Wrapper(id, busChain);
        this.signals = new Pigeon2GyroRecords.GyroPigeon2Signals(
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
        gyro.getConfigurator().apply(Pigeon2GyroConstants.PIGEON_2_CONFIGURATION);
    }

    private void optimizeBusAndSignalOfGyro() {
        BaseStatusSignal.setUpdateFrequencyForAll(
                50,
                signals.X_ACCELERATION_SIGNAL(),
                signals.Y_ACCELERATION_SIGNAL(),
                signals.Z_ACCELERATION_SIGNAL()
        );

        signals.YAW_SIGNAL().setUpdateFrequency(PoseEstimatorConstants.ODOMETRY_FREQUENCY_HERTZ);
        signals.PITCH_SIGNAL().setUpdateFrequency(100);

        gyro.optimizeBusUtilization();
    }

    protected Pigeon2Wrapper getGyro() {
        return gyro;
    }

    protected Pigeon2GyroRecords.GyroPigeon2Signals getSignals() {
        return signals;
    }

}
