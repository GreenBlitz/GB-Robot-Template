package frc.robot.subsystems.swerve.modules.talonfx;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import frc.robot.constants.GlobalConstants;
import frc.robot.subsystems.swerve.modules.ModuleID;
import frc.robot.superstructers.poseestimator.PoseEstimatorConstants;
import frc.utils.devicewrappers.TalonFXWrapper;

public class TalonFXModuleConfigObject {

    private final TalonFXWrapper driveMotor, steerMotor;
    private final CANcoder encoder;
    private final TalonFXModuleSignals moduleSignals;

    public TalonFXModuleConfigObject(
            TalonFXConfiguration steerConfiguration, TalonFXConfiguration driveConfiguration,
            CANcoderConfiguration encoderConfiguration, ModuleID moduleID
    ) {
        this.encoder = new CANcoder(moduleID.encoderDeviceID().ID(), moduleID.encoderDeviceID().busChain().getChainName());
        this.steerMotor = new TalonFXWrapper(moduleID.steerMotorDeviceID());
        this.driveMotor =  new TalonFXWrapper(moduleID.driveMotorDeviceID());

        this.moduleSignals = new TalonFXModuleSignals(
                encoder.getAbsolutePosition(),
                encoder.getVelocity(),
                encoder.getSupplyVoltage(),

                steerMotor.getPosition(),
                steerMotor.getVelocity(),
                steerMotor.getAcceleration(),
                steerMotor.getMotorVoltage(),

                driveMotor.getPosition(),
                driveMotor.getVelocity(),
                driveMotor.getAcceleration(),
                driveMotor.getMotorVoltage(),
                driveMotor.getStatorCurrent()
        );

        configEncoder(encoderConfiguration);
        optimizeBusAndSignalOfEncoder();

        configSteerMotor(steerConfiguration);
        steerMotor.setInverted(moduleID.isSteerMotorInverted());
        optimizeBusAndSignalOfSteerMotor();

        configDriveMotor(driveConfiguration);
        driveMotor.setInverted(moduleID.isDriveMotorInverted());
        optimizeBusAndSignalOfDriveMotor();
    }

    private void configEncoder(CANcoderConfiguration encoderConfiguration) {
        MagnetSensorConfigs magnetSensorConfigs = new MagnetSensorConfigs();
        encoder.getConfigurator().refresh(magnetSensorConfigs);
        encoderConfiguration.MagnetSensor.MagnetOffset = magnetSensorConfigs.MagnetOffset;
        encoder.getConfigurator().apply(encoderConfiguration);
    }
    private void optimizeBusAndSignalOfEncoder() {
        BaseStatusSignal.setUpdateFrequencyForAll(
                PoseEstimatorConstants.ODOMETRY_FREQUENCY_HERTZ,
                moduleSignals.encoderAbsolutePositionSignal()
        );
        BaseStatusSignal.setUpdateFrequencyForAll(
                GlobalConstants.DEFAULT_SIGNALS_FREQUENCY_HERTZ,
                moduleSignals.encoderVelocitySignal(),
                moduleSignals.encoderVoltageSignal()
        );

        encoder.optimizeBusUtilization();
    }

    private void configSteerMotor(TalonFXConfiguration steerConfiguration) {
        steerConfiguration.Feedback.FeedbackRemoteSensorID = encoder.getDeviceID();
        steerMotor.applyConfiguration(steerConfiguration);
    }
    private void optimizeBusAndSignalOfSteerMotor() {
        BaseStatusSignal.setUpdateFrequencyForAll(
                PoseEstimatorConstants.ODOMETRY_FREQUENCY_HERTZ,
                moduleSignals.steerMotorPositionSignal(),
                moduleSignals.steerMotorVelocitySignal(),
                moduleSignals.steerMotorAccelerationSignal()
        );
        BaseStatusSignal.setUpdateFrequencyForAll(GlobalConstants.DEFAULT_SIGNALS_FREQUENCY_HERTZ, moduleSignals.steerMotorVoltageSignal());

        steerMotor.optimizeBusUtilization();
    }

    private void configDriveMotor(TalonFXConfiguration driveConfiguration) {
        driveMotor.applyConfiguration(driveConfiguration);
    }
    private void optimizeBusAndSignalOfDriveMotor() {
        BaseStatusSignal.setUpdateFrequencyForAll(
                PoseEstimatorConstants.ODOMETRY_FREQUENCY_HERTZ,
                moduleSignals.drivePositionSignal(),
                moduleSignals.driveVelocitySignal(),
                moduleSignals.driveAccelerationSignal()
        );
        BaseStatusSignal.setUpdateFrequencyForAll(
                GlobalConstants.DEFAULT_SIGNALS_FREQUENCY_HERTZ,
                moduleSignals.driveVoltageSignal(),
                moduleSignals.driveStatorCurrentSignal()
        );

        driveMotor.optimizeBusUtilization();
    }


    protected CANcoder getEncoder() {
        return encoder;
    }

    protected TalonFXWrapper getSteerMotor() {
        return steerMotor;
    }

    protected TalonFXWrapper getDriveMotor() {
        return driveMotor;
    }

    protected TalonFXModuleSignals getSignals() {
        return moduleSignals;
    }

}
