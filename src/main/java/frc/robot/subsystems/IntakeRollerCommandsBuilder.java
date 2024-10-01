package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import java.util.function.DoubleSupplier;


public class IntakeRollerCommandsBuilder {

	private IntakeRoller intakeRoller;

	public IntakeRollerCommandsBuilder(IntakeRoller intake) {
		this.intakeRoller = intake;
	}

	public Command moveByPower(double power) {
		return new FunctionalCommand(() -> intakeRoller.setPower(power), () -> {}, interrupted -> intakeRoller.stop(), () -> false, intakeRoller)
			.withName("" + power);
	}

	public Command moveByPower(DoubleSupplier power) {
		return new FunctionalCommand(
			() -> {},
			() -> intakeRoller.setPower(power.getAsDouble()),
			interrupted -> intakeRoller.stop(),
			() -> false,
			intakeRoller
		).withName("");
	}

}

