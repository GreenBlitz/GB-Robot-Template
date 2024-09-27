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
	private final IntakeCommandsBuilder commandBuilder;

	public Intake(IntakeStuff intakeStuff) {
		super(intakeStuff.logPath());
		this.motor = intakeStuff.motor();
		this.digitalInput = intakeStuff.digitalInput();
		this.intakeStuff = intakeStuff;
		this.digitalInputsInputs = new DigitalInputInputsAutoLogged();

		this.commandBuilder = new IntakeCommandsBuilder(this);
		updateInputs();
	}

	public IntakeCommandsBuilder getCommandsBuilder() {
		return commandBuilder;
	}

	@Override
	public String getLogPath() {
		return intakeStuff.logPath();
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
		Logger.recordOutput(intakeStuff.logPath() + "IsObjectIn", isObjectIn());
	}

	protected void setPower(double power) {
		motor.setPower(power);
	}

	protected void stop() {
		motor.stop();
	}

}
