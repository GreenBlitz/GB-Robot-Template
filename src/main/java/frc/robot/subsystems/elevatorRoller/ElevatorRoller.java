package frc.robot.subsystems.elevatorRoller;

import frc.robot.hardware.digitalinput.DigitalInputInputsAutoLogged;
import frc.robot.hardware.digitalinput.IDigitalInput;
import frc.robot.hardware.motor.IMotor;
import frc.utils.GBSubsystem;
import org.littletonrobotics.junction.Logger;

public class ElevatorRoller extends GBSubsystem {

	private final IMotor motor;
	private final IDigitalInput elevatorRollerDigitalInput;
	private final DigitalInputInputsAutoLogged elevatorRollerDigitalInputInputs;
	private final ElevatorRollerAvatiach elevatorRollerAvatiach;
	private final ElevatorRollerCommandBuilder commandBuilder;

	public ElevatorRoller(ElevatorRollerAvatiach elevatorRollerAvatiach) {
		super(elevatorRollerAvatiach.logPath());
		this.motor = elevatorRollerAvatiach.motor();
		this.elevatorRollerDigitalInput = elevatorRollerAvatiach.digitalInput();
		this.elevatorRollerAvatiach = elevatorRollerAvatiach;
		this.elevatorRollerDigitalInputInputs = new DigitalInputInputsAutoLogged();

		this.commandBuilder = new ElevatorRollerCommandBuilder(this);
		updateInputs();
	}

	public ElevatorRollerCommandBuilder getCommandBuilder() {
		return commandBuilder;
	}

	public void updateInputs() {
		elevatorRollerDigitalInput.updateInputs(elevatorRollerDigitalInputInputs);
	}

	public boolean isNoteIn() {
		return elevatorRollerDigitalInputInputs.debouncedValue;
	}

	@Override
	protected void subsystemPeriodic() {
		updateInputs();
		Logger.processInputs(elevatorRollerAvatiach.digitalInputLogPath(), elevatorRollerDigitalInputInputs);
		Logger.recordOutput(elevatorRollerAvatiach.logPath() + "IsNoteInElevatorRoller", isNoteIn());
	}

	protected void setPower(double power) {
		motor.setPower(power);
	}

	protected void stop() {
		motor.stop();
	}

	protected void setBrake(boolean brake) {
		motor.setBrake(brake);
	}

}
