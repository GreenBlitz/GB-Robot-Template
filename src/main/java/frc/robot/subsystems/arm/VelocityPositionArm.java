package frc.robot.subsystems.arm;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.hardware.interfaces.ControllableMotor;
import frc.robot.hardware.interfaces.IRequest;
import frc.robot.hardware.interfaces.VelocityPositionRequest;
import org.littletonrobotics.junction.Logger;

public class VelocityPositionArm extends Arm {

	private final VelocityPositionRequest velocityPositionRequest;
	private Rotation2d targetPosition;

	public VelocityPositionArm(
		String logPath,
		ControllableMotor motor,
		ArmSignals signals,
		IRequest<Double> voltageRequest,
		VelocityPositionRequest velocityPositionRequest,
		double kG
	) {
		super(logPath, motor, signals, voltageRequest, velocityPositionRequest, kG);
		this.velocityPositionRequest = velocityPositionRequest;
		this.targetPosition = Rotation2d.kZero;
	}

	public void setTargetPositionVelocity(Rotation2d targetPosition, Rotation2d targetVelocity) {
		velocityPositionRequest.withSetPoint(targetPosition);
		velocityPositionRequest.setVelocity(targetVelocity);
		this.targetPosition = targetPosition;
		motor.applyRequest(velocityPositionRequest);
	}

	@Override
	public void setTargetPosition(Rotation2d targetPosition) {
		setTargetPositionVelocity(targetPosition, Rotation2d.kZero);
	}

	@Override
	public void log() {
		super.log();
		Logger.recordOutput(getLogPath() + "/VelocityPositionArmTargetVelocity", velocityPositionRequest.getVelocityRPS());
		Logger.recordOutput(getLogPath() + "/PositionTarget", velocityPositionRequest.getSetPoint());
	}

}
