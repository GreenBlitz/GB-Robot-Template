package frc.robot.subsystems.elevator;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.hardware.interfaces.*;
import org.littletonrobotics.junction.Logger;


public class DynamicMotionMagicElevator extends Elevator {

	private final IDynamicMotionMagicRequest dynamicMotionMagicRequest;
	private final double defaultDynamicMotionAccelerationMeters;
	private final double defaultDynamicMotionVelocityMeters;
	private final DynamicMotionMagicElevatorCommandsBuilder dynamicMotionMagicCommandsBuilder;

	public DynamicMotionMagicElevator(
		String logPath,
		ControllableMotor elevator,
		ElevatorSignals elevatorSignals,
		IRequest<Double> elevatorVoltageRequest,
		IDynamicMotionMagicRequest motionMagicRequest,
		double defaultMotionMagicAcceleration,
		double defaultMotionMagicVelocity,
		double kG,
		double drumDiameter,
		double minHeight,
		double maxHeight

	) {
		super(logPath, elevator, motionMagicRequest, elevatorVoltageRequest, elevatorSignals, kG, drumDiameter, minHeight, maxHeight);
		this.dynamicMotionMagicRequest = motionMagicRequest;
		this.defaultDynamicMotionAccelerationMeters = defaultMotionMagicAcceleration;
		this.defaultDynamicMotionVelocityMeters = defaultMotionMagicVelocity;
		this.dynamicMotionMagicCommandsBuilder = new DynamicMotionMagicElevatorCommandsBuilder(this);
		setDefaultCommand(getCommandsBuilder().stayPut());
	}

	public DynamicMotionMagicElevatorCommandsBuilder getCommandsBuilder() {
		return dynamicMotionMagicCommandsBuilder;
	}

	public void setTargetPosition(double target, double acceleration, double velocity) {
		dynamicMotionMagicRequest.withSetPoint(metersToAngle(target));
		dynamicMotionMagicRequest.withMaxAccelerationRPS(metersToAngle(acceleration));
		dynamicMotionMagicRequest.withMaxVelocityRPS(metersToAngle(velocity));
		motor.applyRequest(dynamicMotionMagicRequest);
	}

	public void setTargetPosition(Rotation2d target, double acceleration, double velocity) {
		dynamicMotionMagicRequest.withSetPoint(target);
		dynamicMotionMagicRequest.withMaxAccelerationRPS(metersToAngle(acceleration));
		dynamicMotionMagicRequest.withMaxVelocityRPS(metersToAngle(velocity));
		motor.applyRequest(dynamicMotionMagicRequest);
	}


	@Override
	public void setTargetPosition(double target) {
		setTargetPosition(target, defaultDynamicMotionAccelerationMeters, defaultDynamicMotionVelocityMeters);
	}

	@Override
	public void setTargetPosition(Rotation2d target) {
		setTargetPosition(target, defaultDynamicMotionAccelerationMeters, defaultDynamicMotionVelocityMeters);
	}

	@Override
	public void log() {
		super.log();
		Logger.recordOutput(getLogPath() + "/DynamicMotionMagicAcceleration", dynamicMotionMagicRequest.getMaxAccelerationRPS());
		Logger.recordOutput(getLogPath() + "/DynamicMotionMagicVelocity", dynamicMotionMagicRequest.getMaxVelocityRPS());
	}

}
