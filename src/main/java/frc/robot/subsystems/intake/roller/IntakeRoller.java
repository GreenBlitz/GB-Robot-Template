package frc.robot.subsystems.intake.roller;

import frc.robot.hardware.digitalinput.DigitalInputInputsAutoLogged;
import frc.robot.hardware.digitalinput.IDigitalInput;
import frc.robot.hardware.motor.IMotor;
import frc.robot.subsystems.GBSubsystem;
import org.littletonrobotics.junction.Logger;

public class IntakeRoller extends GBSubsystem {

	private final IMotor motor;
	private final IDigitalInput beamBreaker;
	private final DigitalInputInputsAutoLogged beamBreakerInputs;
	private final IntakeRollerStuff intakeRollerStuff;
	private final IntakeRollerCommandsBuilder intakeRollercommandsBuilder;

	public IntakeRoller(IntakeRollerStuff intakeRollerStuff) {
		super(intakeRollerStuff.logPath());

		this.motor = intakeRollerStuff.motor();
		this.beamBreaker = intakeRollerStuff.digitalInput();
		this.intakeRollerStuff = intakeRollerStuff;
		this.beamBreakerInputs = new DigitalInputInputsAutoLogged();
		this.intakeRollercommandsBuilder = new IntakeRollerCommandsBuilder(this);

		updateInputs();
	}

	public IntakeRollerCommandsBuilder getCommandsBuilder() {
		return intakeRollercommandsBuilder;
	}

	public boolean isNoteIn() {
		return beamBreakerInputs.debouncedValue;
	}

	protected void stop() {
		motor.stop();
	}

	protected void setPower(double power) {
		motor.setPower(power);
	}

	public void updateInputs() {
		beamBreaker.updateInputs(beamBreakerInputs);
		motor.updateSignals(intakeRollerStuff.voltageSignal());
		Logger.processInputs(intakeRollerStuff.digitalInputLogPath(), beamBreakerInputs);
		Logger.recordOutput(intakeRollerStuff.logPath() + "IsNoteIn", isNoteIn());
	}

	@Override
	protected void subsystemPeriodic() {
		updateInputs();
	}

}
