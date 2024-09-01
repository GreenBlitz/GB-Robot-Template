package frc.utils.devicewrappers;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;
import frc.utils.PIDObject;

public class SparkMaxWrapper extends CANSparkMax {

	public SparkMaxWrapper(int deviceId, MotorType type) {
		super(deviceId, type);
	}

	public void applyConfiguration(SparkMaxConfiguration config) {
		super.restoreFactoryDefaults();

		configPID(config.slot0);
		configPID(config.slot1);
		configPID(config.slot2);
		configPID(config.slot3);

		super.getEncoder().setPositionConversionFactor(config.conversionFactor);
		super.getEncoder().setVelocityConversionFactor(config.conversionFactor);

		super.setSoftLimit(CANSparkBase.SoftLimitDirection.kForward, (float) config.forwardAngleSoftLimit.getRotations());
		super.enableSoftLimit(CANSparkBase.SoftLimitDirection.kForward, config.enableForwardSoftLimit);

		super.setSoftLimit(CANSparkBase.SoftLimitDirection.kReverse, (float) config.backwardAngleSoftLimit.getRotations());
		super.enableSoftLimit(CANSparkBase.SoftLimitDirection.kReverse, config.enableForwardSoftLimit);

		super.setSmartCurrentLimit(config.currentLimit);

		super.burnFlash();
	}

	public void configPID(PIDObject pid) {
		super.getPIDController().setP(pid.getP(), pid.getSlot());
		super.getPIDController().setI(pid.getI(), pid.getSlot());
		super.getPIDController().setD(pid.getD(), pid.getSlot());
		super.getPIDController().setFF(pid.getFeedforward(), pid.getSlot());
		super.getPIDController().setDFilter(pid.getDFilter(), pid.getSlot());
		super.getPIDController().setIZone(pid.getIActivationErrorTolerance(), pid.getSlot());
		super.getPIDController().setOutputRange(pid.getMinPower(), pid.getMaxPower());
	}

}
