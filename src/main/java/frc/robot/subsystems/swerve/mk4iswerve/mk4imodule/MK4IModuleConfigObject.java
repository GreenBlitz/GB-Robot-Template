package frc.robot.subsystems.swerve.mk4iswerve.mk4imodule;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import frc.utils.devicewrappers.GBTalonFXPro;

public class MK4IModuleConfigObject {

    private final GBTalonFXPro driveMotor, steerMotor;
    private final CANcoder steerEncoder;
    protected StatusSignal<Double> steerEncoderAbsolutePositionSignal, steerEncoderVelocitySignal, steerEncoderVoltageSignal;
    protected StatusSignal<Double> steerPositionSignal, steerVelocitySignal, steerAccelerationSignal, steerVoltageSignal;
    protected StatusSignal<Double> driveStatorCurrentSignal, drivePositionSignal, driveVelocitySignal, driveAccelerationSignal, driveVoltageSignal;

    protected MK4IModuleConfigObject(String busChain, int steerMotorId, boolean isSteerMotorInverted, int driveMotorId, boolean isDriveMotorInverted, int steerEncoderId) {
        this.steerEncoder = new CANcoder(steerEncoderId, busChain);
        this.driveMotor = new GBTalonFXPro(driveMotorId, busChain);
        this.steerMotor = new GBTalonFXPro(steerMotorId, busChain);

        configEncoder();
        optimizeBusAndSignalOfEncoder();

        configDriveMotor();
        driveMotor.setInverted(isDriveMotorInverted);
        optimizeBusAndSignalOfDriveMotor();

        configSteerMotor();
        steerMotor.setInverted(isSteerMotorInverted);
        optimizeBusAndSignalOfSteerMotor();
    }

    private void configEncoder() {
        MagnetSensorConfigs magnetSensorConfigs = new MagnetSensorConfigs();
        steerEncoder.getConfigurator().refresh(magnetSensorConfigs);
        MK4IModuleConstants.ENCODER_CONFIG.MagnetSensor.MagnetOffset = magnetSensorConfigs.MagnetOffset;
        steerEncoder.getConfigurator().apply(MK4IModuleConstants.ENCODER_CONFIG);
    }

    private void optimizeBusAndSignalOfEncoder() {
        steerEncoderAbsolutePositionSignal = steerEncoder.getAbsolutePosition();
        steerEncoderVelocitySignal = steerEncoder.getVelocity();
        steerEncoderVoltageSignal = steerEncoder.getSupplyVoltage();

        BaseStatusSignal.setUpdateFrequencyForAll(
                250,
                steerEncoderAbsolutePositionSignal
        );
        BaseStatusSignal.setUpdateFrequencyForAll(
                100,
                steerEncoderVelocitySignal,
                steerEncoderVoltageSignal
        );

        steerEncoder.optimizeBusUtilization();
    }

    private void configDriveMotor() {
        driveMotor.applyConfiguration(MK4IModuleConstants.DRIVE_MOTOR_CONFIG);
    }

    private void optimizeBusAndSignalOfDriveMotor() {
        drivePositionSignal = driveMotor.getPosition();
        driveVelocitySignal = driveMotor.getVelocity();
        driveStatorCurrentSignal = driveMotor.getStatorCurrent();
        driveVoltageSignal = driveMotor.getMotorVoltage();
        driveAccelerationSignal = driveMotor.getAcceleration();

        BaseStatusSignal.setUpdateFrequencyForAll(
                250,
                drivePositionSignal,
                driveVelocitySignal,
                driveAccelerationSignal
        );
        BaseStatusSignal.setUpdateFrequencyForAll(
                100,
                driveVoltageSignal,
                driveStatorCurrentSignal
        );

        driveMotor.optimizeBusUtilization();
    }

    private void configSteerMotor() {
        TalonFXConfiguration configuration = MK4IModuleConstants.STEER_MOTOR_CONFIG;
        configuration.Feedback.FeedbackRemoteSensorID = steerEncoder.getDeviceID();
        steerMotor.applyConfiguration(configuration);
    }

    private void optimizeBusAndSignalOfSteerMotor() {
        steerPositionSignal = steerMotor.getPosition();
        steerVelocitySignal = steerMotor.getVelocity();
        steerAccelerationSignal = steerMotor.getVelocity();
        steerVoltageSignal = steerMotor.getMotorVoltage();

        BaseStatusSignal.setUpdateFrequencyForAll(
                250,
                steerPositionSignal,
                steerVelocitySignal,
                steerAccelerationSignal
        );
        BaseStatusSignal.setUpdateFrequencyForAll(
                20,
                steerVoltageSignal
        );

        steerMotor.optimizeBusUtilization();
    }


    public CANcoder getSteerEncoder() {
        return steerEncoder;
    }


    public GBTalonFXPro getDriveMotor() {
        return driveMotor;
    }

    public GBTalonFXPro getSteerMotor() {
        return steerMotor;
    }

}
