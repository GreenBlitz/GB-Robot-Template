package frc.robot.subsystems.solenoid;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import frc.utils.utilcommands.InitExecuteCommand;

import java.util.function.DoubleSupplier;

public class SolenoidCommandsBuilder {

	private final Solenoid solenoid;

	public SolenoidCommandsBuilder(Solenoid solenoid) {
		this.solenoid = solenoid;
	}

	//@formatter:off
	public Command setPower(double power) {
		return new FunctionalCommand(
			() -> {},
			() -> solenoid.setPower(power),
			interrupted -> solenoid.stop(),
			() -> false,
			solenoid
		);
	}

	public Command setPower(DoubleSupplier powerSupplier) {
		return new FunctionalCommand(
			() -> {},
			() -> solenoid.setPower(powerSupplier.getAsDouble()),
			interrupted -> solenoid.stop(),
			() -> false,
			solenoid
		);
	}

	public Command stop() {
		return new InitExecuteCommand(() -> {}, solenoid::stop, solenoid);
	}
	//@formatter:on

}
