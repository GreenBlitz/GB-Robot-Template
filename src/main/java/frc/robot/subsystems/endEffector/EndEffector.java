package frc.robot.subsystems.endEffector;

import frc.robot.hardware.digitalinput.DigitalInputInputsAutoLogged;
import frc.robot.hardware.digitalinput.IDigitalInput;
import frc.robot.hardware.interfaces.IMotor;
import frc.robot.subsystems.GBSubsystem;
import org.littletonrobotics.junction.Logger;

public class EndEffector extends GBSubsystem {

	private final IMotor roller;
	private final IDigitalInput frontBeamBreaker;
	private final DigitalInputInputsAutoLogged frontBeamBreakerInputs;
	private final IDigitalInput backBeamBreaker;
	private final DigitalInputInputsAutoLogged backBeamBreakerInputs;
	private final EndEffectorCommandsBuilder commandsBuilder;

	public EndEffector(String logPath, IMotor roller, IDigitalInput frontBeamBreaker, IDigitalInput backBeamBreaker) {
		super(logPath);
		this.roller = roller;

		this.frontBeamBreaker = frontBeamBreaker;
		this.frontBeamBreakerInputs = new DigitalInputInputsAutoLogged();

		this.backBeamBreaker = backBeamBreaker;
		this.backBeamBreakerInputs = new DigitalInputInputsAutoLogged();

		this.commandsBuilder = new EndEffectorCommandsBuilder(this);

		periodic();
	}

	public EndEffectorCommandsBuilder getCommandsBuilder() {
		return commandsBuilder;
	}

	public boolean isCoralInFront() {
		return frontBeamBreakerInputs.debouncedValue;
	}

	public boolean isCoralInBack() {
		return backBeamBreakerInputs.debouncedValue;
	}

	@Override
	protected void subsystemPeriodic() {
		updateInputs();
		log();
	}

	private void updateInputs() {
		frontBeamBreaker.updateInputs(frontBeamBreakerInputs);
		backBeamBreaker.updateInputs(backBeamBreakerInputs);
		Logger.processInputs(EndEffectorConstants.LOG_PATH + "FrontBeamBreaker/", frontBeamBreakerInputs);
		Logger.processInputs(EndEffectorConstants.LOG_PATH + "BackBeamBreaker/", backBeamBreakerInputs);
	}

	private void log() {
		Logger.recordOutput(EndEffectorConstants.LOG_PATH + "FrontBeamBreaker/", isCoralInFront());
		Logger.recordOutput(EndEffectorConstants.LOG_PATH + "BackBeamBreaker/", isCoralInBack());
	}

	public void stop() {
		roller.stop();
	}

	public void setPower(double power) {
		roller.setPower(power);
	}

	public void setBrake(boolean brake) {
		roller.setBrake(brake);
	}

}
