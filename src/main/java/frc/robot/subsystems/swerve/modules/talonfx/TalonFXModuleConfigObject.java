package frc.robot.subsystems.swerve.modules.talonfx;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import frc.robot.constants.GlobalConstants;
import frc.robot.subsystems.swerve.modules.ModuleID;
import frc.robot.poseestimation.PoseEstimatorConstants;
import frc.utils.devicewrappers.TalonFXWrapper;

class TalonFXModuleConfigObject {

    private final TalonFXWrapper driveMotor, steerMotor;
    private final CANcoder encoder;
    private final TalonFXModuleSignals moduleSignals;

    protected TalonFXModuleConfigObject(
            TalonFXConfiguration steerConfiguration, TalonFXConfiguration driveConfiguration,
            CANcoderConfiguration encoderConfiguration, ModuleID moduleID
    ) {
        this.encoder = new CANcoder(moduleID.encoderDeviceID().ID(), moduleID.encoderDeviceID().busChain().getChainName());
        this.steerMotor = new TalonFXWrapper(moduleID.steerMotorDeviceID());
        this.driveMotor =  new TalonFXWrapper(moduleID.driveMotorDeviceID());

        this.moduleSignals = new TalonFXModuleSignals(
                encoder.getPosition().clone(),
                encoder.getVelocity().clone(),
                encoder.getSupplyVoltage().clone(),

                steerMotor.getPosition().clone(),
                steerMotor.getVelocity().clone(),
                steerMotor.getAcceleration().clone(),
                steerMotor.getMotorVoltage().clone(),

                driveMotor.getPosition().clone(),
                driveMotor.getVelocity().clone(),
                driveMotor.getAcceleration().clone(),
                driveMotor.getMotorVoltage().clone(),
                driveMotor.getStatorCurrent().clone()
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
                moduleSignals.encoderPositionSignal()
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
