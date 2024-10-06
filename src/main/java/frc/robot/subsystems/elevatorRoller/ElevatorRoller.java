package frc.robot.subsystems.elevatorRoller;

import frc.robot.hardware.digitalinput.DigitalInputInputsAutoLogged;
import frc.robot.hardware.digitalinput.IDigitalInput;
import frc.robot.hardware.motor.IMotor;
import frc.robot.subsystems.GBSubsystem;
import org.littletonrobotics.junction.Logger;

public class ElevatorRoller extends GBSubsystem {

	private final IMotor motor;
	private final IDigitalInput elevatorRollerDigitalInput;
	private final DigitalInputInputsAutoLogged elevatorRollerDigitalInputInputs;
	private final ElevatorRollerStuff elevatorRollerStuff;
	private final ElevatorRollerCommandsBuilder commandsBuilder;

	public ElevatorRoller(ElevatorRollerStuff elevatorRollerStuff) {
		super(elevatorRollerStuff.logPath());
		this.motor = elevatorRollerStuff.motor();
		this.elevatorRollerDigitalInput = elevatorRollerStuff.digitalInput();
		this.elevatorRollerStuff = elevatorRollerStuff;
		this.elevatorRollerDigitalInputInputs = new DigitalInputInputsAutoLogged();
		this.commandsBuilder = new ElevatorRollerCommandsBuilder(this);

		updateInputs();
	}

	public ElevatorRollerCommandsBuilder getCommandsBuilder() {
		return commandsBuilder;
	}

	public boolean isNoteIn() {
		return elevatorRollerDigitalInputInputs.debouncedValue;
	}

	protected void stop() {
		motor.stop();
	}

	protected void setPower(double power) {
		motor.setPower(power);
	}

	public void setBrake(boolean brake) {
		motor.setBrake(brake);
	}

	public void updateInputs() {
		elevatorRollerDigitalInput.updateInputs(elevatorRollerDigitalInputInputs);
		motor.updateSignals(elevatorRollerStuff.motorVoltage());
		Logger.processInputs(elevatorRollerStuff.digitalInputLogPath(), elevatorRollerDigitalInputInputs);
		Logger.recordOutput(elevatorRollerStuff.logPath() + "IsNoteInElevatorRoller", isNoteIn());
	}

	@Override
	protected void subsystemPeriodic() {
		updateInputs();
	}

}
