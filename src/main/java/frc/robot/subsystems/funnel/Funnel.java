package frc.robot.subsystems.funnel;

import frc.robot.hardware.digitalinput.DigitalInputInputsAutoLogged;
import frc.robot.hardware.digitalinput.IDigitalInput;
import frc.robot.hardware.motor.IMotor;
import frc.robot.hardware.signal.InputSignal;
import frc.robot.subsystems.GBSubsystem;
import org.littletonrobotics.junction.Logger;

public class Funnel extends GBSubsystem {

	private final IMotor motor;
	private final IDigitalInput leftDigitalInput;
	private final IDigitalInput rightDigitalInput;
	private final DigitalInputInputsAutoLogged leftDigitalInputsInputs;
	private final DigitalInputInputsAutoLogged rightDigitalInputsInputs;
	private final InputSignal<Double> voltageSignal;
	private final FunnelCommandsBuilder commandBuilder;
	private final FunnelStuff funnelStuff;

	public Funnel(FunnelStuff funnelStuff) {
		super(funnelStuff.logPath());
		this.funnelStuff = funnelStuff;
		this.motor = funnelStuff.motor();
		this.leftDigitalInput = funnelStuff.leftDigitalInput();
		this.rightDigitalInput = funnelStuff.rightDigitalInput();
		this.voltageSignal = funnelStuff.inputSignal();
		this.leftDigitalInputsInputs = new DigitalInputInputsAutoLogged();
		this.rightDigitalInputsInputs = new DigitalInputInputsAutoLogged();
		this.commandBuilder = new FunnelCommandsBuilder(this);

		updateInputs();
	}

	public FunnelCommandsBuilder getCommandsBuilder() {
		return commandBuilder;
	}

	public boolean isObjectIn() {
		return leftDigitalInputsInputs.debouncedValue || rightDigitalInputsInputs.debouncedValue;
	}

	protected void setPower(double power) {
		motor.setPower(power);
	}

	protected void stop() {
		motor.stop();
	}

	public void setBrake(boolean brake) {
		motor.setBrake(brake);
	}

	public void updateInputs() {
		leftDigitalInput.updateInputs(leftDigitalInputsInputs);
		rightDigitalInput.updateInputs(rightDigitalInputsInputs);
		motor.updateSignals(voltageSignal);
		Logger.processInputs(funnelStuff.leftDigitalInputLogPath(), leftDigitalInputsInputs);
		Logger.processInputs(funnelStuff.rightDigitalInputLogPath(), rightDigitalInputsInputs);
		Logger.recordOutput(funnelStuff.logPath() + "IsObjectIn", isObjectIn());
	}

	@Override
	protected void subsystemPeriodic() {
		updateInputs();
	}

}
