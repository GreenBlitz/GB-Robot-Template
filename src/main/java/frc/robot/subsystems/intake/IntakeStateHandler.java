package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.Command;

public class IntakeStateHandler {

	private final Intake intake;

	public IntakeStateHandler(Intake intake) {
		this.intake = intake;
	}

	public Command setState(IntakeState intakeState) {
		return intake.getCommandsBuilder().setPower(intakeState.getPower());
	}

}
