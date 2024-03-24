package frc.robot.subsystems.newswerve.mk4iswerve;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import frc.robot.subsystems.newswerve.SwerveUtils;
import frc.utils.devicewrappers.GBTalonFXPro;

public class MK4IModuleConfigObject {

    private final GBTalonFXPro driveMotor, steerMotor;
    private final CANcoder steerEncoder;
    public StatusSignal<Double> steerEncoderPositionSignal, steerEncoderVelocitySignal, steerEncoderVoltageSignal;
    public StatusSignal<Double> steerPositionSignal, steerVelocitySignal, steerVoltageSignal;
    public StatusSignal<Double> driveStatorCurrentSignal, drivePositionSignal, driveVelocitySignal, driveVoltageSignal;


    protected MK4IModuleConfigObject(String busChain, int steerMotorId, int driveMotorId, int steerEncoderId)
    {
        this.driveMotor = new GBTalonFXPro(steerMotorId, busChain);
        this.steerMotor = new GBTalonFXPro(driveMotorId, busChain);
        this.steerEncoder = new CANcoder(steerEncoderId, busChain);
        configEncoder();
        optimizeBusAndSignalOfEncoder();
        configSteerMotor();
        optimizeBusAndSignalOfSteerMotor();
        configDriveMotor();
        optimizeBusAndSignalOfDriveMotor();
    }

    private void configEncoder(){
        steerEncoder.getConfigurator().apply(MK4IModuleConstants.ENCODER_CONFIG);
    }

    private void optimizeBusAndSignalOfEncoder(){
        steerEncoderPositionSignal = steerEncoder.getPosition();
        steerEncoderVelocitySignal = steerEncoder.getVelocity();
        steerEncoderVoltageSignal = steerEncoder.getSupplyVoltage();

        BaseStatusSignal.setUpdateFrequencyForAll(
                100,
                steerEncoderPositionSignal,
                steerEncoderVelocitySignal,
                steerEncoderVoltageSignal
        );

        steerEncoder.optimizeBusUtilization();
    }

    private void configSteerMotor(){
        TalonFXConfiguration configuration = MK4IModuleConstants.STEER_MOTOR_CONFIG;
        configuration.Feedback.FeedbackRemoteSensorID = steerEncoder.getDeviceID();
        steerMotor.applyConfiguration(configuration);
    }

    private void optimizeBusAndSignalOfSteerMotor(){
        steerPositionSignal = steerMotor.getPosition();
        steerVelocitySignal = steerMotor.getVelocity();
        steerVoltageSignal = steerMotor.getMotorVoltage();

        steerMotor.updateFrequency(
                250,
                steerPositionSignal,
                steerVelocitySignal
        );
        steerMotor.updateFrequency(
                20,
                steerVoltageSignal
        );

        steerMotor.optimizeBusUtilization();
    }

    private void configDriveMotor(){
        driveMotor.applyConfiguration(MK4IModuleConstants.DRIVE_MOTOR_CONFIG);
    }

    private void optimizeBusAndSignalOfDriveMotor(){
        drivePositionSignal = driveMotor.getPosition();
        driveVelocitySignal = driveMotor.getVelocity();
        driveStatorCurrentSignal = driveMotor.getStatorCurrent();
        driveVoltageSignal = driveMotor.getMotorVoltage();

        driveMotor.updateFrequency(
                250,
                drivePositionSignal,
                driveVelocitySignal
        );
        driveMotor.updateFrequency(
                100,
                driveVoltageSignal,
                driveStatorCurrentSignal
        );

        driveMotor.optimizeBusUtilization();
    }

    public CANcoder getSteerEncoder() {
        return steerEncoder;
    }

    public GBTalonFXPro getSteerMotor(){
        return steerMotor;
    }

    public GBTalonFXPro getDriveMotor() {
        return driveMotor;
    }
}
