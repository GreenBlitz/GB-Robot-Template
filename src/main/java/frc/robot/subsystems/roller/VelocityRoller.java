package frc.robot.subsystems.roller;

import frc.utils.costomUnits.Angle2d;
import frc.robot.hardware.interfaces.IRequest;
import frc.robot.hardware.interfaces.InputSignal;
import frc.robot.hardware.interfaces.ControllableMotor;
import org.littletonrobotics.junction.Logger;

public class VelocityRoller extends Roller {

	private final InputSignal<Angle2d> velocitySignal;
	private final IRequest<Angle2d> velocityRequest;
	private final VelocityRollerCommandsBuilder commandsBuilder;

	public VelocityRoller(
		String logPath,
		ControllableMotor motor,
		InputSignal<Double> voltageSignal,
		InputSignal<Double> currentSignal,
		InputSignal<Angle2d> positionSignal,
		InputSignal<Angle2d> velocitySignal,
		IRequest<Double> voltageRequest,
		IRequest<Angle2d> velocityRequest
	) {
		super(logPath, motor, voltageSignal, currentSignal, positionSignal, velocitySignal, voltageRequest);
		this.velocitySignal = velocitySignal;
		this.velocityRequest = velocityRequest;
		this.commandsBuilder = new VelocityRollerCommandsBuilder(this);
		setDefaultCommand(commandsBuilder.stop());
	}

	@Override
	public VelocityRollerCommandsBuilder getCommandsBuilder() {
		return commandsBuilder;
	}

	public Angle2d getVelocity() {
		return velocitySignal.getLatestValue();
	}

	public void setVelocity(Angle2d targetVelocity) {
		motor.applyRequest(velocityRequest.withSetPoint(targetVelocity));
	}

	@Override
	public void stop() {
		super.stop();
		velocityRequest.withSetPoint(Angle2d.ZERO);
	}

	@Override
	public void update() {
		super.update();
		Logger.recordOutput(getLogPath() + "/TargetVelocity", velocityRequest.getSetPoint());
		motor.updateInputs(velocitySignal);
	}

}
