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

		configPID(config.slot0, 0);
		configPID(config.slot1, 1);
		configPID(config.slot2, 2);
		configPID(config.slot3, 3);

		super.getEncoder().setPositionConversionFactor(config.conversionFactor);
		super.getEncoder().setVelocityConversionFactor(config.conversionFactor);

		super.setSoftLimit(CANSparkBase.SoftLimitDirection.kForward, (float) config.forwardAngleLimit.getRotations());
		super.enableSoftLimit(CANSparkBase.SoftLimitDirection.kForward, config.enableForwardSoftLimit);

		super.setSoftLimit(CANSparkBase.SoftLimitDirection.kReverse, (float) config.backwardAngleLimit.getRotations());
		super.enableSoftLimit(CANSparkBase.SoftLimitDirection.kReverse, config.enableForwardSoftLimit);

		super.setSmartCurrentLimit(config.currentLimit);

		super.burnFlash();
	}

	public void configPID(PIDObject pid, int slot) {
		super.getPIDController().setP(pid.getP(), slot);
		super.getPIDController().setI(pid.getI(), slot);
		super.getPIDController().setD(pid.getD(), slot);
		super.getPIDController().setFF(pid.getFF(), slot);
		super.getPIDController().setDFilter(pid.getDFilter(), slot);
		super.getPIDController().setIZone(pid.getIZone(), slot);
		super.getPIDController().setOutputRange(pid.getMinPower(), pid.getMaxPower());
	}

}
