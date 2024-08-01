package frc.robot.subsystems.swerve.modules.check.drive.talonfx;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import frc.robot.constants.GlobalConstants;
import frc.robot.poseestimation.PoseEstimatorConstants;
import frc.utils.ctre.CTREDeviceID;
import frc.utils.devicewrappers.TalonFXWrapper;

public class TalonFXDriveConfigObject {

    private final TalonFXWrapper driveMotor;
    private final TalonFXDriveSignals signals;

    public TalonFXDriveConfigObject(CTREDeviceID driveMotorID, TalonFXConfiguration configuration) {
        this.driveMotor = new TalonFXWrapper(driveMotorID);
        this.signals = new TalonFXDriveSignals(
                driveMotor.getPosition().clone(),
                driveMotor.getVelocity().clone(),
                driveMotor.getAcceleration().clone(),
                driveMotor.getMotorVoltage().clone(),
                driveMotor.getStatorCurrent().clone()
        );

        configDriveMotor(configuration);
        optimizeBusAndSignalOfDriveMotor();
    }


    private void configDriveMotor(TalonFXConfiguration driveConfiguration) {
        driveMotor.applyConfiguration(driveConfiguration);
    }

    private void optimizeBusAndSignalOfDriveMotor() {
        BaseStatusSignal.setUpdateFrequencyForAll(
                PoseEstimatorConstants.ODOMETRY_FREQUENCY_HERTZ,
                signals.drivePositionSignal(),
                signals.driveVelocitySignal(),
                signals.driveAccelerationSignal()
        );
        BaseStatusSignal.setUpdateFrequencyForAll(
                GlobalConstants.DEFAULT_SIGNALS_FREQUENCY_HERTZ,
                signals.driveVoltageSignal(),
                signals.driveStatorCurrentSignal()
        );

        driveMotor.optimizeBusUtilization();
    }


    public TalonFXWrapper getDriveMotor() {
        return driveMotor;
    }

    public TalonFXDriveSignals getSignals() {
        return signals;
    }

}
