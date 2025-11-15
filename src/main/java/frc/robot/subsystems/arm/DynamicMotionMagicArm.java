package frc.robot.subsystems.arm;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.hardware.interfaces.*;
import org.littletonrobotics.junction.Logger;


public class DynamicMotionMagicArm extends Arm {

	private final IDynamicMotionMagicRequest dynamicMotionMagicRequest;
	private final Rotation2d defaultDynamicMotionAcceleration;
	private final Rotation2d defaultDynamicMotionVelocity;
	private final DynamicMotionMagicArmCommandBuilder dynamicMotionMagicCommandBuilder;

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
		this.dynamicMotionMagicCommandBuilder = new DynamicMotionMagicArmCommandBuilder(this);
        setDefaultCommand(getCommandsBuilder().stayInPlace());
	}

	public DynamicMotionMagicArmCommandBuilder getCommandsBuilder() {
		return dynamicMotionMagicCommandBuilder;
	}

	public void setTargetPosition(Rotation2d target, Rotation2d acceleration, Rotation2d velocity, double arbitraryFeedForward) {
		dynamicMotionMagicRequest.withSetPoint(target);
		dynamicMotionMagicRequest.withMaxAccelerationRotation2dPerSecondSquared(acceleration);
		dynamicMotionMagicRequest.withMaxVelocityRotation2dPerSecond(velocity);
		dynamicMotionMagicRequest.withArbitraryFeedForward(arbitraryFeedForward);
		motor.applyRequest(dynamicMotionMagicRequest);
	}

	@Override
	public void setTargetPosition(Rotation2d target) {
		dynamicMotionMagicRequest.withSetPoint(target);
		dynamicMotionMagicRequest.withMaxAccelerationRotation2dPerSecondSquared(defaultDynamicMotionAcceleration);
		dynamicMotionMagicRequest.withMaxVelocityRotation2dPerSecond(defaultDynamicMotionVelocity);
		motor.applyRequest(dynamicMotionMagicRequest);
	}

	@Override
	public void log() {
		Logger.recordOutput(getLogPath() + "/PositionTarget", dynamicMotionMagicRequest.getSetPoint());
		Logger.recordOutput(getLogPath() + "/DynamicMotionMagicAcceleration", dynamicMotionMagicRequest.getMaxAccelerationRotation2dPerSecondSquared());
		Logger.recordOutput(getLogPath() + "/DynamicMotionMagicVelocity", dynamicMotionMagicRequest.getMaxVelocityRotation2dPerSecond());
	}

}
