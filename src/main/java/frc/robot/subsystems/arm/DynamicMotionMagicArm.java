package frc.robot.subsystems.arm;

import org.wpilib.math.geometry.Angle2d;
import frc.robot.hardware.interfaces.*;
import org.littletonrobotics.junction.Logger;


public class DynamicMotionMagicArm extends Arm {

	private final IDynamicMotionMagicRequest dynamicMotionMagicRequest;
	private final Angle2d defaultDynamicMotionAcceleration;
	private final Angle2d defaultDynamicMotionVelocity;
	private final DynamicMotionMagicArmCommandsBuilder dynamicMotionMagicCommandsBuilder;

	public DynamicMotionMagicArm(
		String logPath,
		ControllableMotor arm,
		ArmSignals signals,
		IRequest<Double> armVoltageRequest,
		IDynamicMotionMagicRequest motionMagicRequest,
		Angle2d defaultMotionMagicAcceleration,
		Angle2d defaultMotionMagicVelocity,
		double kG
	) {
		super(logPath, arm, signals, armVoltageRequest, motionMagicRequest, kG);
		this.dynamicMotionMagicRequest = motionMagicRequest;
		this.defaultDynamicMotionAcceleration = defaultMotionMagicAcceleration;
		this.defaultDynamicMotionVelocity = defaultMotionMagicVelocity;
		this.dynamicMotionMagicCommandsBuilder = new DynamicMotionMagicArmCommandsBuilder(this);
		setDefaultCommand(getCommandsBuilder().stayInPlace());
	}

	public DynamicMotionMagicArmCommandsBuilder getCommandsBuilder() {
		return dynamicMotionMagicCommandsBuilder;
	}

	public void setTargetPosition(Angle2d target, Angle2d acceleration, Angle2d velocity) {
		dynamicMotionMagicRequest.withSetPoint(target);
		dynamicMotionMagicRequest.withMaxAccelerationRPS(acceleration);
		dynamicMotionMagicRequest.withMaxVelocityRPS(velocity);
		motor.applyRequest(dynamicMotionMagicRequest);
	}

	@Override
	public void setTargetPosition(Angle2d target) {
		setTargetPosition(target, defaultDynamicMotionAcceleration, defaultDynamicMotionVelocity);
	}

	@Override
	public void log() {
		super.log();
		Logger.recordOutput(getLogPath() + "/DynamicMotionMagicAcceleration", dynamicMotionMagicRequest.getMaxAccelerationRPS());
		Logger.recordOutput(getLogPath() + "/DynamicMotionMagicVelocity", dynamicMotionMagicRequest.getMaxVelocityRPS());
	}

}
