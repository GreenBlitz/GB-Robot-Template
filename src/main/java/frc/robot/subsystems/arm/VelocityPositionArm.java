package frc.robot.subsystems.arm;

import frc.robot.hardware.interfaces.ControllableMotor;
import frc.robot.hardware.interfaces.IRequest;
import frc.robot.hardware.interfaces.VelocityPositionRequest;
import frc.utils.costomUnits.Angle2d;
import org.littletonrobotics.junction.Logger;

public class VelocityPositionArm extends Arm {

	private final VelocityPositionRequest velocityPositionRequest;

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
	}

	public void setTargetPositionVelocity(Angle2d targetPosition, Angle2d targetVelocity) {
		velocityPositionRequest.withSetPoint(targetPosition);
		velocityPositionRequest.setVelocity(targetVelocity);
		motor.applyRequest(velocityPositionRequest);
	}

	@Override
	public void setTargetPosition(Angle2d targetPosition) {
		setTargetPositionVelocity(targetPosition, Angle2d.ZERO);
	}

	@Override
	public void log() {
		super.log();
		Logger.recordOutput(getLogPath() + "/TargetVelocity", velocityPositionRequest.getVelocityRPS());
	}

}
