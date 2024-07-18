package frc.robot.subsystems.swerve.gyro.pigeon2;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import frc.robot.constants.GlobalConstants;
import frc.robot.superstructers.poseestimator.PoseEstimatorConstants;
import frc.utils.ctre.CTREDeviceID;
import frc.utils.devicewrappers.Pigeon2Wrapper;

public class Pigeon2GyroConfigObject {

    private final Pigeon2Wrapper gyro;

    private final Pigeon2GyroSignals signals;

    public Pigeon2GyroConfigObject(CTREDeviceID pigeon2DeviceID, Pigeon2Configuration configuration) {
        this.gyro = new Pigeon2Wrapper(pigeon2DeviceID);
        this.signals = new Pigeon2GyroSignals(
                gyro.getYaw(),
                gyro.getAccelerationX(),
                gyro.getAccelerationY(),
                gyro.getAccelerationZ()
        );

        configGyro(configuration);
        optimizeBusAndSignalOfGyro();
    }


    private void configGyro(Pigeon2Configuration configuration) {
        gyro.getConfigurator().apply(configuration);
    }

    private void optimizeBusAndSignalOfGyro() {
        BaseStatusSignal.setUpdateFrequencyForAll(
                GlobalConstants.DEFAULT_SIGNALS_FREQUENCY_HERTZ,
                signals.xAccelerationSignal(),
                signals.yAccelerationSignal(),
                signals.zAccelerationSignal()
        );
        signals.yawSignal().setUpdateFrequency(PoseEstimatorConstants.ODOMETRY_FREQUENCY_HERTZ);

        gyro.optimizeBusUtilization();
    }


    protected Pigeon2Wrapper getGyro() {
        return gyro;
    }

    protected Pigeon2GyroSignals getSignals() {
        return signals;
    }

}
