package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;

public class IntakeCommandsBuilder {

	private final Intake intake;

	public IntakeCommandsBuilder(Intake intake) {
		this.intake = intake;
	}

	public Command moveByPower(double power) {
		return new FunctionalCommand(() -> intake.setPower(power), () -> {}, interrupted -> intake.stop(), () -> false, intake)
			.withName("Move by power: " + power);
	}

}
