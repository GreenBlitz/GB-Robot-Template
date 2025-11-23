package frc.robot.statemachine.superstructure;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class IntakeStateHandler {

	public IntakeStateHandler() {}

	public Command setState(IntakeState intakeState) {
		return new InstantCommand();
	}

}
