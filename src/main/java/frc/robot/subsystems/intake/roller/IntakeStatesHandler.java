package frc.robot.subsystems.intake.roller;

import edu.wpi.first.wpilibj2.command.Command;

public class IntakeStatesHandler {

	private final IntakeRoller intakeRoller;

	public IntakeStatesHandler(IntakeRoller intakeRoller) {
		this.intakeRoller = intakeRoller;
	}

	public Command setState(IntakeStates intakeStates) {
		return intakeRoller.getCommandsBuilder().setPower(intakeStates.getPower());
	}

}
