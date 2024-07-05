package frc.robot.subsystems.swerve.gyro.pigeon2;

import com.ctre.phoenix6.BaseStatusSignal;
import frc.robot.constants.RobotConstants;
import frc.robot.superstructers.poseestimator.PoseEstimatorConstants;
import frc.utils.ctre.CTREDeviceID;
import frc.utils.devicewrappers.Pigeon2Wrapper;

class Pigeon2GyroConfigObject {

    private final Pigeon2Wrapper gyro;

    private final Pigeon2GyroRecords.GyroPigeon2Signals signals;

    protected Pigeon2GyroConfigObject(CTREDeviceID pigeon2DeviceID) {
        this.gyro = new Pigeon2Wrapper(pigeon2DeviceID);
        this.signals = new Pigeon2GyroRecords.GyroPigeon2Signals(
                gyro.getYaw(),
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
                RobotConstants.DEFAULT_SIGNALS_FREQUENCY_HERTZ,
                signals.X_ACCELERATION_SIGNAL(),
                signals.Y_ACCELERATION_SIGNAL(),
                signals.Z_ACCELERATION_SIGNAL()
        );
        signals.YAW_SIGNAL().setUpdateFrequency(PoseEstimatorConstants.ODOMETRY_FREQUENCY_HERTZ);

        gyro.optimizeBusUtilization();
    }


    protected Pigeon2Wrapper getGyro() {
        return gyro;
    }

    protected Pigeon2GyroRecords.GyroPigeon2Signals getSignals() {
        return signals;
    }

}
