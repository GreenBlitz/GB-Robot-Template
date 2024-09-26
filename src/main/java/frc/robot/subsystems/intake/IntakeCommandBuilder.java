package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;

public class IntakeCommandBuilder {
	
	private final Intake intake;
	
	public IntakeCommandBuilder(Intake intake) {
		this.intake = intake;
	}
	
	public Command moveByPower(double power) {
		return new FunctionalCommand(
				() -> intake.setPower(power),
				() -> {},
				interrupted -> intake.stop(),
				() -> false,
				intake
		).withName("Move By Power: " + power);
	}

}
