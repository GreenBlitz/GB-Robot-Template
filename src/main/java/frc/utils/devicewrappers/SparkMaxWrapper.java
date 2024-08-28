package frc.utils.devicewrappers;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;

public class SparkMaxWrapper extends CANSparkMax {

	public SparkMaxWrapper(int deviceId, MotorType type) {
		super(deviceId, type);
	}

	public void applyConfiguration(SparkMaxConfiguration configuration) {
		this.restoreFactoryDefaults();

		this.getEncoder().setPositionConversionFactor(configuration.conversionFactor);
		this.getEncoder().setVelocityConversionFactor(configuration.conversionFactor);

		this.setSoftLimit(CANSparkBase.SoftLimitDirection.kForward, (float) configuration.forwardAngleLimit.getRotations());
		this.enableSoftLimit(CANSparkBase.SoftLimitDirection.kForward, configuration.enableForwardSoftLimit);

		this.setSoftLimit(CANSparkBase.SoftLimitDirection.kReverse, (float) configuration.backwardAngleLimit.getRotations());
		this.enableSoftLimit(CANSparkBase.SoftLimitDirection.kReverse, configuration.enableForwardSoftLimit);

		this.setSmartCurrentLimit(configuration.currentLimit);

		this.getPIDController().setP(configuration.slot0.getkP(), 0);
		this.getPIDController().setI(configuration.slot0.getkI(), 0);
		this.getPIDController().setD(configuration.slot0.getkD(), 0);
		this.getPIDController().setDFilter(configuration.slot0.getDFilter(), 0);
		this.getPIDController().setFF(configuration.slot0.getFF(), 0);
		this.getPIDController().setIZone(configuration.slot0.getiZone(), 0);
		this.getPIDController().setOutputRange(configuration.slot0.getOutputRangeMin(), configuration.slot0.getOutputRangeMax(), 0);
		this.getPIDController().setSmartMotionMaxVelocity(configuration.slot0.getSmartMotionMaxVelocity(), 0);
		this.getPIDController().setSmartMotionMaxAccel(configuration.slot0.getSmartMotionMaxAcceleration(), 0);
		this.getPIDController().setSmartMotionMinOutputVelocity(configuration.slot0.getSmartMotionMinOutputVelocity(), 0);
		this.getPIDController().setSmartMotionAllowedClosedLoopError(configuration.slot0.getSmartMotionAllowedClosedLoopError(), 0);
		this.getPIDController().setSmartMotionAccelStrategy(configuration.slot0.getSmartMotionAccelStrategy(), 0);
		this.getPIDController().setIMaxAccum(configuration.slot0.getiMaxAccumulator(), 0);
		this.getPIDController().setIAccum(configuration.slot0.getiAccumulator());
		this.getPIDController().setPositionPIDWrappingEnabled(configuration.slot0.isPositionPIDWrappingEnabled());
		this.getPIDController().setPositionPIDWrappingMinInput(configuration.slot0.getPositionPIDWrappingMinInput());
		this.getPIDController().setPositionPIDWrappingMaxInput(configuration.slot0.getPositionPIDWrappingMaxInput());

		this.burnFlash();
	}
}
