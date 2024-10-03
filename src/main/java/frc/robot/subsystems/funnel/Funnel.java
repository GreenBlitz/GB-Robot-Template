package frc.robot.subsystems.funnel;

import frc.robot.hardware.digitalinput.DigitalInputInputsAutoLogged;
import frc.robot.hardware.digitalinput.IDigitalInput;
import frc.robot.hardware.motor.IMotor;
import frc.robot.hardware.signal.InputSignal;
import frc.utils.GBSubsystem;
import org.littletonrobotics.junction.Logger;

public class Funnel extends GBSubsystem {

	private final IMotor motor;
	private final IDigitalInput digitalInput;
	private final DigitalInputInputsAutoLogged digitalInputsInputs;
	private final InputSignal<Double> voltageSignal;
	private final FunnelCommandsBuilder commandBuilder;
	private final FunnelStuff funnelStuff;

	public Funnel(FunnelStuff funnelStuff) {
		super(funnelStuff.logPath());
		this.funnelStuff = funnelStuff;
		this.motor = funnelStuff.motor();
		this.digitalInput = funnelStuff.digitalInput();
		this.voltageSignal = funnelStuff.inputSignal();
		this.digitalInputsInputs = new DigitalInputInputsAutoLogged();
		this.commandBuilder = new FunnelCommandsBuilder(this);

		updateInputs();
	}

	public FunnelCommandsBuilder getCommandsBuilder() {
		return commandBuilder;
	}

	public boolean isObjectIn() {
		return digitalInputsInputs.debouncedValue;
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
		digitalInput.updateInputs(digitalInputsInputs);
		motor.updateSignals(voltageSignal);
		Logger.processInputs(funnelStuff.digitalInputLogPath(), digitalInputsInputs);
		Logger.recordOutput(funnelStuff.logPath() + "IsObjectIn", isObjectIn());
	}

	@Override
	protected void subsystemPeriodic() {
		updateInputs();
	}

}
