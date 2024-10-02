package frc.robot.subsystems.funnel;

import frc.robot.hardware.digitalinput.DigitalInputInputsAutoLogged;
import frc.robot.hardware.digitalinput.IDigitalInput;
import frc.robot.hardware.motor.IMotor;
import frc.utils.GBSubsystem;
import org.littletonrobotics.junction.Logger;

public class Funnel extends GBSubsystem {

	private final IMotor motor;
	private final FunnelStuff funnelStuff;
	private final FunnelCommandsBuilder commandsBuilder;
	private final IDigitalInput shooterDigitalInput;
	private final IDigitalInput elevatorDigitalInput;
	private final DigitalInputInputsAutoLogged shooterDigitalInputInputs;
	private final DigitalInputInputsAutoLogged elevatorDigitalInputInputs;

	public Funnel(FunnelStuff funnelStuff) {
		super(funnelStuff.logPath());
		this.motor = funnelStuff.motor();
		this.shooterDigitalInput = funnelStuff.shooterDigitalInput();
		this.elevatorDigitalInput = funnelStuff.ampDigitalInput();
		this.funnelStuff = funnelStuff;
		this.shooterDigitalInputInputs = new DigitalInputInputsAutoLogged();
		this.elevatorDigitalInputInputs = new DigitalInputInputsAutoLogged();

		this.commandsBuilder = new FunnelCommandsBuilder(this);

		updateInputs();
	}

	public FunnelCommandsBuilder getCommandsBuilder() {
		return commandsBuilder;
	}

	public boolean isNoteInShooter() {
		return shooterDigitalInputInputs.debouncedValue;
	}

	public boolean isNoteInElevator() {
		return elevatorDigitalInputInputs.debouncedValue;
	}

	public void setBrake(boolean brake) {
		motor.setBrake(brake);
	}

	public void stop() {
		motor.stop();
	}

	public void setPower(double power) {
		motor.setPower(power);
	}

	public void updateInputs() {
		shooterDigitalInput.updateInputs(shooterDigitalInputInputs);
		elevatorDigitalInput.updateInputs(elevatorDigitalInputInputs);
		motor.updateSignals(funnelStuff.voltageSignal(), funnelStuff.positionSignal());
	}

	@Override
	protected void subsystemPeriodic() {
		updateInputs();
		Logger.processInputs(funnelStuff.shooterDigitalInputLogPath(), shooterDigitalInputInputs);
		Logger.processInputs(funnelStuff.ampDigitalInputLogPath(), elevatorDigitalInputInputs);
		Logger.recordOutput("isNoteInElevator", elevatorDigitalInputInputs.debouncedValue);
		Logger.recordOutput("isNoteInShooter", shooterDigitalInputInputs.debouncedValue);
	}

}
