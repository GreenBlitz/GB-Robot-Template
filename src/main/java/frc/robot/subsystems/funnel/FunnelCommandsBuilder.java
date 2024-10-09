package frc.robot.subsystems.funnel;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;

import java.util.function.DoubleSupplier;

public class FunnelCommandsBuilder {

	private final Funnel funnel;

	public FunnelCommandsBuilder(Funnel funnel) {
		this.funnel = funnel;
	}

	// @formatter:off
	public Command setPower(double power) {
		return new FunctionalCommand(
			() -> {},
			() -> funnel.setPower(power),
			interrupted -> funnel.stop(),
			() -> false,
			funnel
		).withName("Set power: " + power);
	}

	public Command setPower(DoubleSupplier power) {
		return new FunctionalCommand(
			() -> {},
			() -> funnel.setPower(power.getAsDouble()),
			interrupted -> funnel.stop(),
			() -> false,
			funnel
		).withName("Set power by supplier");
	}

	public Command stop() {
		return new RunCommand(funnel::stop, funnel).withName("Stop");
	}
	// @formatter:on

}
