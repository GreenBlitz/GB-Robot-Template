package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;

import java.util.function.DoubleSupplier;

public class IntakeCommandsBuilder {

	private final Intake intake;

	public IntakeCommandsBuilder(Intake intake) {
		this.intake = intake;
	}

	public Command moveByPower(double power) {
		return new FunctionalCommand(() -> intake.setPower(power), () -> {}, interrupted -> intake.stop(), () -> false, intake)
			.withName("Move by power: " + power);
	}

	public Command moveByPower(DoubleSupplier power) {
		return new FunctionalCommand(() -> {}, () -> intake.setPower(power.getAsDouble()), interrupted -> intake.stop(), () -> false, intake)
			.withName("Move power supplier");
	}

}
