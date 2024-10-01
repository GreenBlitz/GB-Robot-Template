package frc.robot.subsystems.funnel;

import edu.wpi.first.math.geometry.Rotation2d;
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
	private final IDigitalInput ampDigitalInput;
	private final DigitalInputInputsAutoLogged shooterDigitalInputInputs;
	private final DigitalInputInputsAutoLogged ampDigitalInputInputs;

	public Funnel(FunnelStuff funnelStuff) {
		super(funnelStuff.logPath());
		this.motor = funnelStuff.motor();
		this.shooterDigitalInput = funnelStuff.shooterDigitalInput();
		this.ampDigitalInput = funnelStuff.ampDigitalInput();
		this.funnelStuff = funnelStuff;
		this.shooterDigitalInputInputs = new DigitalInputInputsAutoLogged();
		this.ampDigitalInputInputs = new DigitalInputInputsAutoLogged();

		commandsBuilder = new FunnelCommandsBuilder(this);
		updateInputs();
	}

	public FunnelCommandsBuilder getCommandsBuilder(){
		return commandsBuilder;
	}

	public boolean isNoteInShooter() {
		return shooterDigitalInputInputs.debouncedValue;
	}

	public boolean isNoteInAmp() {
		return ampDigitalInputInputs.debouncedValue;
	}

	public void updateInputs() {
		shooterDigitalInput.updateInputs(shooterDigitalInputInputs);
		ampDigitalInput.updateInputs(ampDigitalInputInputs);
		motor.updateSignals(funnelStuff.voltageSignal(), funnelStuff.positionSignal());
	}

	public void setPower(double power) {
		motor.setPower(power);
	}

	public void stop() {
		motor.stop();
	}

	public void setBrake(boolean brake) {
		motor.setBrake(brake);
	}

	@Override
	protected void subsystemPeriodic() {
		updateInputs();
		Logger.processInputs(funnelStuff.shooterDigitalInputLogPath(), shooterDigitalInputInputs);
		Logger.processInputs(funnelStuff.ampDigitalInputLogPath(), ampDigitalInputInputs);
	}

}
