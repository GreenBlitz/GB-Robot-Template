package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;

import java.util.function.DoubleSupplier;

public class IntakeRollerCommandsBuilder {

	private IntakeRoller intakeRoller;

	public IntakeRollerCommandsBuilder(IntakeRoller intake) {
		this.intakeRoller = intake;
	}

	public Command setPower(double Power) {
		return new FunctionalCommand(() -> {}, () -> intakeRoller.setPower(Power), interrupted -> intakeRoller.stop(), () -> false, intakeRoller)
			.withName("set power: " + Power);
	}

	public Command setPower(DoubleSupplier Power) {
		return new FunctionalCommand(
			() -> {},
			() -> intakeRoller.setPower(Power.getAsDouble()),
			interrupted -> intakeRoller.stop(),
			() -> false,
			intakeRoller
		).withName("set power: " + Power);
	}

	public Command stop() {
		return new InstantCommand(intakeRoller::stop, intakeRoller).withName("stop");
	}

}

