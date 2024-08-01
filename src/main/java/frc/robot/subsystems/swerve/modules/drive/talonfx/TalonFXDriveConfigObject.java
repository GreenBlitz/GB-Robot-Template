package frc.robot.subsystems.swerve.modules.drive.talonfx;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import frc.robot.constants.GlobalConstants;
import frc.robot.poseestimation.PoseEstimatorConstants;
import frc.utils.ctre.CTREDeviceID;
import frc.utils.devicewrappers.TalonFXWrapper;

class TalonFXDriveConfigObject {

    private final TalonFXWrapper driveMotor;
    private final TalonFXDriveSignals signals;

    protected TalonFXDriveConfigObject(CTREDeviceID driveMotorID, boolean inverted, TalonFXConfiguration configuration) {
        this.driveMotor = new TalonFXWrapper(driveMotorID);
        this.signals = new TalonFXDriveSignals(
                driveMotor.getPosition().clone(),
                driveMotor.getVelocity().clone(),
                driveMotor.getAcceleration().clone(),
                driveMotor.getMotorVoltage().clone(),
                driveMotor.getStatorCurrent().clone()
        );

        configDriveMotor(configuration);
        driveMotor.setInverted(inverted);
        optimizeBusAndSignalOfDriveMotor();
    }


    private void configDriveMotor(TalonFXConfiguration driveConfiguration) {
        driveMotor.applyConfiguration(driveConfiguration);
    }

    private void optimizeBusAndSignalOfDriveMotor() {
        BaseStatusSignal.setUpdateFrequencyForAll(
                PoseEstimatorConstants.ODOMETRY_FREQUENCY_HERTZ,
                signals.positionSignal(),
                signals.velocitySignal(),
                signals.accelerationSignal()
        );
        BaseStatusSignal.setUpdateFrequencyForAll(
                GlobalConstants.DEFAULT_SIGNALS_FREQUENCY_HERTZ,
                signals.voltageSignal(),
                signals.statorCurrentSignal()
        );

        driveMotor.optimizeBusUtilization();
    }


    protected TalonFXWrapper getDriveMotor() {
        return driveMotor;
    }

    protected TalonFXDriveSignals getSignals() {
        return signals;
    }

}
