package frc.robot.subsystems.swerve.modules.mk4i;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import frc.robot.constants.GlobalConstants;
import frc.robot.subsystems.swerve.modules.ModuleID;
import frc.robot.superstructers.poseestimator.PoseEstimatorConstants;
import frc.utils.devicewrappers.TalonFXWrapper;

public class MK4IModuleConfigObject {

    private final TalonFXWrapper driveMotor, steerMotor;
    private final CANcoder encoder;
    private final MK4IModuleSignals moduleSignals;

    public MK4IModuleConfigObject(
            TalonFXConfiguration steerConfiguration, TalonFXConfiguration driveConfiguration,
            CANcoderConfiguration encoderConfiguration, ModuleID moduleID
    ) {
        this.encoder = new CANcoder(moduleID.encoderDeviceID().ID(), moduleID.encoderDeviceID().busChain().getChainName());
        this.steerMotor = new TalonFXWrapper(moduleID.steerMotorDeviceID());
        this.driveMotor =  new TalonFXWrapper(moduleID.driveMotorDeviceID());

        this.moduleSignals = new MK4IModuleSignals(
                encoder.getAbsolutePosition(),
                encoder.getVelocity(),
                encoder.getSupplyVoltage(),

                driveMotor.getPosition(),
                driveMotor.getVelocity(),
                driveMotor.getAcceleration(),
                driveMotor.getMotorVoltage(),
                driveMotor.getStatorCurrent(),

                steerMotor.getPosition(),
                steerMotor.getVelocity(),
                steerMotor.getAcceleration(),
                steerMotor.getMotorVoltage()
        );

        configEncoder(encoderConfiguration);
        optimizeBusAndSignalOfEncoder();

        configDriveMotor(driveConfiguration);
        driveMotor.setInverted(moduleID.isDriveMotorInverted());
        optimizeBusAndSignalOfDriveMotor();

        configSteerMotor(steerConfiguration);
        steerMotor.setInverted(moduleID.isSteerMotorInverted());
        optimizeBusAndSignalOfSteerMotor();
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

    protected CANcoder encoder() {
        return encoder;
    }

    protected TalonFXWrapper steerMotor() {
        return steerMotor;
    }

    protected TalonFXWrapper driveMotor() {
        return driveMotor;
    }

    protected MK4IModuleSignals signals() {
        return moduleSignals;
    }

}
