package frc.robot.subsystems.intake;

import frc.robot.hardware.digitalinput.DigitalInputInputsAutoLogged;
import frc.robot.hardware.digitalinput.IDigitalInput;
import frc.robot.hardware.motor.IMotor;
import frc.robot.subsystems.GBSubsystem;
import org.littletonrobotics.junction.Logger;

public class IntakeRoller extends GBSubsystem {

	private final IMotor motor;
	private final IDigitalInput digitalInput;
	private final DigitalInputInputsAutoLogged digitalInputsInputs;
	private final IntakeRollerStuff intakeRollerStuff;
	private final IntakeRollerCommandsBuilder intakeRollercommandsBuilder;

	public IntakeRoller(IntakeRollerStuff intakeRollerStuff) {
		super(intakeRollerStuff.logPath());
		this.motor = intakeRollerStuff.motor();
		this.digitalInput = intakeRollerStuff.digitalInput();
		this.intakeRollerStuff = intakeRollerStuff;
		this.digitalInputsInputs = new DigitalInputInputsAutoLogged();
		this.intakeRollercommandsBuilder = new IntakeRollerCommandsBuilder(this);

		updateInputs();
	}

	public IntakeRollerCommandsBuilder getCommandsBuilder() {
		return intakeRollercommandsBuilder;
	}

	public boolean isNoteIn() {
		return digitalInputsInputs.debouncedValue;
	}

	protected void stop() {
		motor.stop();
	}

	protected void setPower(double power) {
		motor.setPower(power);
	}

	public void updateInputs() {
		digitalInput.updateInputs(digitalInputsInputs);
		motor.updateSignals(intakeRollerStuff.voltageSignal());
		Logger.processInputs(intakeRollerStuff.digitalInputLogPath(), digitalInputsInputs);
		Logger.recordOutput(intakeRollerStuff.logPath() + "IsNoteIn", isNoteIn());
	}

	@Override
	protected void subsystemPeriodic() {
		updateInputs();
	}

}
