package frc.robot.subsystems.arm;

import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import frc.robot.hardware.interfaces.ControllableMotor;
import frc.robot.hardware.interfaces.IFeedForwardRequest;
import frc.robot.hardware.interfaces.IRequest;
import frc.robot.hardware.phoenix6.request.Phoenix6Request;
import org.littletonrobotics.junction.Logger;

public class CurrentControlArm extends Arm {

	private final IRequest<Double> currentRequest;
	private final CurrentControlArmCommandsBuilder commandsBuilder;

	public CurrentControlArm(
		String logPath,
		ControllableMotor motor,
		ArmSignals signals,
		IRequest<Double> voltageRequest,
		IFeedForwardRequest positionRequest,
		IRequest<Double> currentRequest,
		double kG
	) {
		super(logPath, motor, signals, voltageRequest, positionRequest, kG);
		this.currentRequest = currentRequest;
		this.commandsBuilder = new CurrentControlArmCommandsBuilder(this);
	}

	@Override
	public CurrentControlArmCommandsBuilder getCommandsBuilder() {
		return commandsBuilder;
	}

	public void setCurrent(double current) {
		Logger.recordOutput(getLogPath() + "/TargetCurrent", current);
		motor.applyRequest(currentRequest.withSetPoint(current));
	}

	public void setCurrentWithoutLimit(double current) {
		if (currentRequest instanceof Phoenix6Request<Double> phoenix6CurrentRequest) {
			if (phoenix6CurrentRequest.getControlRequest() instanceof TorqueCurrentFOC torqueCurrentRequest) {
				torqueCurrentRequest.IgnoreSoftwareLimits = true;
				setCurrent(current);
				torqueCurrentRequest.IgnoreSoftwareLimits = false;
			}
		}
	}

}
