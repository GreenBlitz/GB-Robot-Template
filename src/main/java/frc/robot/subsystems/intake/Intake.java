package frc.robot.subsystems.intake;

import frc.robot.hardware.digitalinput.DigitalInputInputsAutoLogged;
import frc.robot.hardware.digitalinput.IDigitalInput;
import frc.robot.hardware.motor.IMotor;
import frc.utils.GBSubsystem;
import org.littletonrobotics.junction.Logger;


public class Intake extends GBSubsystem {

	private final IMotor motor;
	private final IDigitalInput digitalInput;
	private final DigitalInputInputsAutoLogged digitalInputsInputs;
	private final IntakeStuff intakeStuff;
	private final IntakeCommandBuilder commandBuilder;

	public Intake(IntakeStuff intakeStuff) {
		super(intakeStuff.logPath());
		this.motor = intakeStuff.motor();
		this.digitalInput = intakeStuff.digitalInput();
		this.intakeStuff = intakeStuff;
		this.digitalInputsInputs = new DigitalInputInputsAutoLogged();

		this.commandBuilder = new IntakeCommandBuilder(this);
		updateInputs();
	}

	public IntakeCommandBuilder getCommandBuilder() {
		return commandBuilder;
	}

	public boolean isObjectIn() {
		return digitalInputsInputs.debouncedValue;
	}

	public void updateInputs() {
		digitalInput.updateInputs(digitalInputsInputs);
		motor.updateSignals(intakeStuff.voltageSignal());
	}

	@Override
	protected void subsystemPeriodic() {
		updateInputs();
		Logger.processInputs(intakeStuff.digitalInputLogPath(), digitalInputsInputs);
	}

	public void setPower(double power) {
		motor.setPower(power);
	}

	public void stop() {
		motor.stop();
	}

}
