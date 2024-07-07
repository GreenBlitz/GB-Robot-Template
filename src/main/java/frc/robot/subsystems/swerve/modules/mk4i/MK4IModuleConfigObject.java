package frc.robot.subsystems.swerve.modules.mk4i;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import frc.robot.constants.GlobalConstants;
import frc.robot.superstructers.poseestimator.PoseEstimatorConstants;
import frc.utils.ctre.CTREDeviceID;
import frc.utils.devicewrappers.TalonFXWrapper;

public class MK4IModuleConfigObject {

    private final TalonFXWrapper driveMotor, steerMotor;
    private final CANcoder encoder;

    private final MK4IModuleRecords.MK4IModuleMotors moduleMotors;
    private final MK4IModuleRecords.MK4IModuleSignals moduleSignals;

    protected MK4IModuleConfigObject(
            CTREDeviceID steerMotorDeviceID, boolean isSteerMotorInverted, TalonFXConfiguration steerConfiguration,
            CTREDeviceID driveMotorDeviceID, boolean isDriveMotorInverted, TalonFXConfiguration driveConfiguration,
            CTREDeviceID encoderID, CANcoderConfiguration encoderConfiguration
    ) {
        this.encoder = new CANcoder(encoderID.ID(), encoderID.busChain().getChainName());
        this.moduleMotors = new MK4IModuleRecords.MK4IModuleMotors(
                new TalonFXWrapper(driveMotorDeviceID),
                new TalonFXWrapper(steerMotorDeviceID)
        );
        this.steerMotor = moduleMotors.steerMotor();
        this.driveMotor = moduleMotors.driveMotor();

        this.moduleSignals = new MK4IModuleRecords.MK4IModuleSignals(
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
        driveMotor.setInverted(isDriveMotorInverted);
        optimizeBusAndSignalOfDriveMotor();

        configSteerMotor(steerConfiguration);
        steerMotor.setInverted(isSteerMotorInverted);
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

    protected CANcoder getEncoder() {
        return encoder;
    }

    protected MK4IModuleRecords.MK4IModuleMotors getMotors() {
        return moduleMotors;
    }

    protected MK4IModuleRecords.MK4IModuleSignals getModuleSignals() {
        return moduleSignals;
    }

}
