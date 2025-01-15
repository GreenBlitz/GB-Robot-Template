package frc.robot.subsystems.endEffector;

import frc.robot.hardware.digitalinput.DigitalInputInputsAutoLogged;
import frc.robot.hardware.digitalinput.IDigitalInput;
import frc.robot.hardware.interfaces.ControllableMotor;
import frc.robot.hardware.interfaces.IRequest;
import frc.robot.subsystems.GBSubsystem;

public class EndEffector extends GBSubsystem {

	private final ControllableMotor roller;
	private final IRequest<Double> powerRequest;
	private final IDigitalInput frontBeamBreaker;
	private final DigitalInputInputsAutoLogged frontBeamBreakerInputs;
	private final IDigitalInput backBeamBreaker;
	private final DigitalInputInputsAutoLogged backBeamBreakerInputs;
	private final EndEffectorCommandsBuilder commandsBuilder;

	public EndEffector(
		ControllableMotor roller,
		IDigitalInput frontBeamBreaker,
		IDigitalInput backBeamBreaker,
		IRequest<Double> powerRequest,
		String logPath
	) {
		super(logPath);
		this.roller = roller;
		this.powerRequest = powerRequest;
		this.frontBeamBreaker = frontBeamBreaker;
		this.frontBeamBreakerInputs = new DigitalInputInputsAutoLogged();
		this.backBeamBreaker = backBeamBreaker;
		this.backBeamBreakerInputs = new DigitalInputInputsAutoLogged();
		this.commandsBuilder = new EndEffectorCommandsBuilder(this);
	}

	public boolean isCoralInFrontBeamBreaker() {
		return frontBeamBreakerInputs.debouncedValue;
	}

	public boolean isCoralInBackBeamBreaker() {
		return backBeamBreakerInputs.debouncedValue;
	}

	public void setPower(double power) {
		roller.applyRequest(powerRequest.withSetPoint(power));
	}

	public void stop() {
		roller.applyRequest(powerRequest.withSetPoint(0.0));
	}

	public EndEffectorCommandsBuilder getCommandsBuilder() {
		return commandsBuilder;
	}

	private void updateInputs() {
		frontBeamBreaker.updateInputs(frontBeamBreakerInputs);
		backBeamBreaker.updateInputs(backBeamBreakerInputs);
	}

	@Override
	protected void subsystemPeriodic() {
		updateInputs();
	}

}
