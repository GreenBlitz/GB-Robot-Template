package frc.robot.subsystems.intake;

import frc.robot.hardware.digitalinput.DigitalInputInputsAutoLogged;
import frc.robot.hardware.digitalinput.IDigitalInput;
import frc.robot.hardware.motor.IMotor;
import frc.utils.GBSubsystem;
import org.littletonrobotics.junction.Logger;


public class IntakeRoller extends GBSubsystem {

	private final IMotor motor;
	private final IDigitalInput digitalInput;
	private final DigitalInputInputsAutoLogged digitalInputsInputs;
	private final frc.robot.subsystems.IntakeRollerStuff intakeRollerStuff;
	private final frc.robot.subsystems.IntakeRollerCommandsBuilder intakeRollercommandBuilder;

	public IntakeRoller(frc.robot.subsystems.IntakeRollerStuff intakeRollerStuff) {
		super(intakeRollerStuff.logPath());
		this.motor = intakeRollerStuff.motor();
		this.digitalInput = intakeRollerStuff.digitalInput();
		this.intakeRollerStuff = intakeRollerStuff;
		this.digitalInputsInputs = new DigitalInputInputsAutoLogged();
		this.intakeRollercommandBuilder = new frc.robot.subsystems.IntakeRollerCommandsBuilder(this);
		updateInputs();
	}

	public frc.robot.subsystems.IntakeRollerCommandsBuilder getCommandsBuilder() {
		return intakeRollercommandBuilder;
	}

	protected void setPower(double power) {
		motor.setPower(power);
	}

	public boolean isObjectIn() {
		return digitalInputsInputs.debouncedValue;
	}

	public void updateInputs() {
		digitalInput.updateInputs(digitalInputsInputs);
		motor.updateSignals(intakeRollerStuff.voltageSignal());
	}

	protected void stop() {
		motor.stop();
	}

	@Override
	protected void subsystemPeriodic() {
		updateInputs();
		Logger.processInputs(intakeRollerStuff.digitalInputLogPath(), digitalInputsInputs);
		Logger.recordOutput(intakeRollerStuff.logPath() + "IsObjectIn", isObjectIn());
	}

}
