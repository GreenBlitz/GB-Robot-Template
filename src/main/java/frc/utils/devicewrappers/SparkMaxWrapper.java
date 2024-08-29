package frc.utils.devicewrappers;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;
import frc.utils.PIDObject;

public class SparkMaxWrapper extends CANSparkMax {

	public SparkMaxWrapper(int deviceId, MotorType type) {
		super(deviceId, type);
	}

	public void applyConfiguration(SparkMaxConfiguration config) {
		this.restoreFactoryDefaults();

		if (config.enableSlot0){
			configPID(config.slot0, 0);
		}
		if (config.enableSlot1){
			configPID(config.slot1, 1);
		}
		if (config.enableSlot2){
			configPID(config.slot2, 2);
		}
		if (config.enableSlot3){
			configPID(config.slot3, 3);
		}

		this.getEncoder().setPositionConversionFactor(config.positionConversionFactor);
		this.getEncoder().setVelocityConversionFactor(config.velocityConversionFactor);

		this.setSoftLimit(CANSparkBase.SoftLimitDirection.kForward, (float) config.forwardAngleLimit.getRotations());
		this.enableSoftLimit(CANSparkBase.SoftLimitDirection.kForward, config.enableForwardSoftLimit);

		this.setSoftLimit(CANSparkBase.SoftLimitDirection.kReverse, (float) config.backwardAngleLimit.getRotations());
		this.enableSoftLimit(CANSparkBase.SoftLimitDirection.kReverse, config.enableForwardSoftLimit);

		this.setSmartCurrentLimit(config.currentLimit);

		this.burnFlash();
	}

	public void configPID(PIDObject pid, int slot){
		super.getPIDController().setP(pid.getKp(), slot);
		super.getPIDController().setI(pid.getKi(), slot);
		super.getPIDController().setD(pid.getKd(), slot);
		super.getPIDController().setFF(pid.getKff(), slot);
		super.getPIDController().setDFilter(pid.getDFilter(), slot);
		super.getPIDController().setIZone(pid.getIZone(), slot);
		super.getPIDController().setOutputRange(-pid.getMaxPower(), pid.getMaxPower());
	}

}
