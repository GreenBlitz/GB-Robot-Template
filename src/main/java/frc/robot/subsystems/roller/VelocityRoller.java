package frc.robot.subsystems.roller;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.hardware.interfaces.IRequest;
import frc.robot.hardware.interfaces.InputSignal;
import frc.robot.hardware.interfaces.ControllableMotor;
import org.littletonrobotics.junction.Logger;

public class VelocityRoller extends Roller {

	private final InputSignal<Rotation2d> velocitySignal;
	private final IRequest<Rotation2d> velocityRequest;
	private final VelocityRollerCommandsBuilder commandsBuilder;

	public VelocityRoller(
		String logPath,
		ControllableMotor motor,
		InputSignal<Double> voltageSignal,
		InputSignal<Double> currentSignal,
		InputSignal<Rotation2d> positionSignal,
		InputSignal<Rotation2d> velocitySignal,
		IRequest<Double> voltageRequest,
		IRequest<Rotation2d> velocityRequest
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

	public Rotation2d getVelocity() {
		return velocitySignal.getLatestValue();
	}

	public void setVelocity(Rotation2d targetVelocity) {
		motor.applyRequest(velocityRequest.withSetPoint(targetVelocity));
	}

	@Override
	public void stop() {
		super.stop();
		velocityRequest.withSetPoint(Rotation2d.kZero);
	}

	@Override
	public void update() {
		super.stop();
		Logger.recordOutput(getLogPath() + "/TargetVelocity", velocityRequest.getSetPoint());
		motor.updateInputs(velocitySignal);
	}

}
