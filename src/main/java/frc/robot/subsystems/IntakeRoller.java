package frc.robot.subsystems;

import frc.robot.hardware.digitalinput.DigitalInputInputsAutoLogged;
import frc.robot.hardware.digitalinput.IDigitalInput;
import frc.robot.hardware.motor.IMotor;
import frc.utils.GBSubsystem;
import org.littletonrobotics.junction.Logger;

public class IntakeRoller extends GBSubsystem {

	private final IMotor motor;
	private final IDigitalInput digitalInput;
	private final DigitalInputInputsAutoLogged digitalInputsInputs;
	private final IntakeRollerStuff intakeRollerStuff;
	private final IntakeRollerCommandsBuilder intakeRollercommandBuilder;

	public IntakeRoller(IntakeRollerStuff intakeRollerStuff) {
		super(intakeRollerStuff.logPath());
		this.motor = intakeRollerStuff.motor();
		this.digitalInput = intakeRollerStuff.digitalInput();
		this.intakeRollerStuff = intakeRollerStuff;
		this.digitalInputsInputs = new DigitalInputInputsAutoLogged();

		this.intakeRollercommandBuilder = new IntakeRollerCommandsBuilder(this);
		updateInputs();
	}

	public IntakeRollerCommandsBuilder getCommandsBuilder() {
		return intakeRollercommandBuilder;
	}

	public boolean isObjectIn() {
		return digitalInputsInputs.debouncedValue;
	}

	public void updateInputs() {
		digitalInput.updateInputs(digitalInputsInputs);
		motor.updateSignals(intakeRollerStuff.voltageSignal());
	}

	@Override
	protected void subsystemPeriodic() {
		updateInputs();
		Logger.processInputs(intakeRollerStuff.digitalInputLogPath(), digitalInputsInputs);
		Logger.recordOutput(intakeRollerStuff.logPath() + "IsObjectIn", isObjectIn());
	}

	protected void setPower(double power) {
		motor.setPower(power);
	}

	protected void stop() {
		motor.stop();
	}

}
