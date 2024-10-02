package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;

public class IntakeRollerCommandsBuilder {

	private IntakeRoller intakeRoller;

	public IntakeRollerCommandsBuilder(IntakeRoller intake) {
		this.intakeRoller = intake;
	}

	public Command setPower(double tagetPower) {
		return new FunctionalCommand(() -> intakeRoller.setPower(tagetPower), () -> {}, interrupted -> intakeRoller.stop(), () -> false, intakeRoller)
			.withName("set power- " + tagetPower);
	}

}

