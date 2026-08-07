package frc.robot.subsystems.arm;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.hardware.interfaces.*;
import org.littletonrobotics.junction.Logger;


public class DynamicMotionMagicArm extends Arm {

	private final IDynamicMotionMagicRequest dynamicMotionMagicRequest;
	private final Rotation2d defaultDynamicMotionAcceleration;
	private final Rotation2d defaultDynamicMotionVelocity;
	private final DynamicMotionMagicArmCommandsBuilder dynamicMotionMagicCommandsBuilder;

	public DynamicMotionMagicArm(
		String logPath,
		ControllableMotor arm,
		ArmSignals signals,
		IRequest<Double> armVoltageRequest,
		IDynamicMotionMagicRequest motionMagicRequest,
		Rotation2d defaultMotionMagicAcceleration,
		Rotation2d defaultMotionMagicVelocity,
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

	public void setTargetPosition(Rotation2d target, Rotation2d acceleration, Rotation2d velocity) {
		dynamicMotionMagicRequest.withSetPoint(target);
		dynamicMotionMagicRequest.withMaxAccelerationRPS(acceleration);
		dynamicMotionMagicRequest.withMaxVelocityRPS(velocity);
		motor.applyRequest(dynamicMotionMagicRequest);
	}

	@Override
	public void setTargetPosition(Rotation2d target) {
		setTargetPosition(target, defaultDynamicMotionAcceleration, defaultDynamicMotionVelocity);
	}

	@Override
	public void log() {
		super.log();
		Logger.recordOutput(getLogPath() + "/DynamicMotionMagicAcceleration", dynamicMotionMagicRequest.getMaxAccelerationRPS());
		Logger.recordOutput(getLogPath() + "/DynamicMotionMagicVelocity", dynamicMotionMagicRequest.getMaxVelocityRPS());
	}

}
