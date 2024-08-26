package frc.utils.devicewrappers;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;

public class SparkMaxWrapper extends CANSparkMax {

    public SparkMaxWrapper(int deviceId, MotorType type) {
        super(deviceId, type);
    }

    public void applyConfiguration(SparkMaxConfiguration configuration){
        this.restoreFactoryDefaults();

        this.getEncoder().setPositionConversionFactor(configuration.conversionFactor);
        this.getEncoder().setVelocityConversionFactor(configuration.conversionFactor);

        this.setSoftLimit(CANSparkBase.SoftLimitDirection.kForward, (float) configuration.forwardAngleLimit.getRotations());
        this.enableSoftLimit(CANSparkBase.SoftLimitDirection.kForward, configuration.enableSoftLimit);

        this.setSoftLimit(CANSparkBase.SoftLimitDirection.kReverse, (float) configuration.backwardAngleLimit.getRotations());
        this.enableSoftLimit(CANSparkBase.SoftLimitDirection.kReverse, configuration.enableSoftLimit);

        this.getPIDController().setP(configuration.pidController.getP());
        this.getPIDController().setI(configuration.pidController.getI());
        this.getPIDController().setD(configuration.pidController.getD());

        this.setSmartCurrentLimit(configuration.currentLimit);

        this.getEncoder().setPosition(configuration.startingPosition.getRotations());

        this.burnFlash();
    }
}
