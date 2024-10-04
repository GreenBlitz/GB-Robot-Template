package frc.robot.subsystems.funnel;

import frc.robot.hardware.digitalinput.DigitalInputInputsAutoLogged;
import frc.robot.hardware.digitalinput.IDigitalInput;
import frc.robot.hardware.motor.IMotor;
import frc.robot.subsystems.GBSubsystem;
import org.littletonrobotics.junction.Logger;

public class Funnel extends GBSubsystem {

	private final IMotor motor;
	private final FunnelStuff funnelStuff;
	private final FunnelCommandsBuilder commandsBuilder;
	private final IDigitalInput shooterDigitalInput;
	private final DigitalInputInputsAutoLogged shooterDigitalInputInputs;

	public Funnel(FunnelStuff funnelStuff) {
		super(funnelStuff.logPath());
		this.motor = funnelStuff.motor();
		this.shooterDigitalInput = funnelStuff.shooterDigitalInput();
		this.funnelStuff = funnelStuff;
		this.shooterDigitalInputInputs = new DigitalInputInputsAutoLogged();
		this.commandsBuilder = new FunnelCommandsBuilder(this);

		updateInputs();
	}

	public FunnelCommandsBuilder getCommandsBuilder() {
		return commandsBuilder;
	}

	public boolean isNoteInShooter() {
		return shooterDigitalInputInputs.debouncedValue;
	}

	protected void stop() {
		motor.stop();
	}

	protected void setPower(double power) {
		motor.setPower(power);
	}

	public void updateInputs() {
		shooterDigitalInput.updateInputs(shooterDigitalInputInputs);
		motor.updateSignals(funnelStuff.voltageSignal());
		Logger.processInputs(funnelStuff.shooterDigitalInputLogPath(), shooterDigitalInputInputs);
		Logger.recordOutput(funnelStuff.shooterDigitalInputLogPath() + "isNoteInShooter", isNoteInShooter());
	}

	@Override
	protected void subsystemPeriodic() {
		updateInputs();
	}

}
