package frc.robot.subsystems.wrist;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;

import java.util.function.DoubleSupplier;

public class WristCommandsBuilder {

	private final Wrist wrist;

	public WristCommandsBuilder(Wrist wrist) {
		this.wrist = wrist;
	}

	//@formatter:off
	public Command setPower(DoubleSupplier power) {
		return new FunctionalCommand(
				() -> {},
				() -> wrist.setPower(power.getAsDouble()),
				interrupted -> wrist.stop(),
				() -> false,
				wrist
		).withName("Set power to: " + power);
	}

	public Command moveToPosition(Rotation2d position) {
		return new FunctionalCommand(
				() -> {},
				() -> wrist.setTargetPosition(position),
				interrupted -> wrist.stayInPlace(),
				() -> false,
				wrist
		).withName("Move to position: " + position);
	}
	//@formatter:on

}
