package frc.robot.subsystems.intake;

import frc.robot.hardware.digitalinput.DigitalInputInputsAutoLogged;
import frc.robot.hardware.digitalinput.IDigitalInput;
import frc.robot.hardware.motor.IMotor;
import frc.robot.hardware.signal.InputSignal;
import frc.utils.GBSubsystem;
import org.littletonrobotics.junction.Logger;


public class Intake extends GBSubsystem {

	private final IMotor motor;
	private final IDigitalInput digitalInput;
	private final DigitalInputInputsAutoLogged digitalInputsInputs;
	private final InputSignal<Double> voltageSignal;
	private final IntakeCommandBuilder commandBuilder;

	public Intake(String logPath, IntakeStuff intakeStuff) {
		super(logPath);
		this.motor = intakeStuff.motor();
		this.digitalInput = intakeStuff.digitalInput();
		this.voltageSignal = intakeStuff.inputSignal();
		this.commandBuilder = new IntakeCommandBuilder(this);

		this.digitalInputsInputs = new DigitalInputInputsAutoLogged();
	}

	public void setPower(double power) {
		motor.setPower(power);
	}

	public void stop() {
		motor.stop();
	}

	public void updateInputs() {
		digitalInput.updateInputs(digitalInputsInputs);
		motor.updateSignals(voltageSignal);
	}

	public IntakeCommandBuilder getCommandBuilder() {
		return commandBuilder;
	}

	@Override
	protected void subsystemPeriodic() {
		updateInputs();
		Logger.processInputs(getLogPath() + "digitalInputs", digitalInputsInputs);
	}

}
