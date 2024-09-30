package frc.robot.subsystems.roller;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;

import java.util.function.DoubleSupplier;

public class RollerCommandsBuilder {

	private final Roller roller;

	public RollerCommandsBuilder(Roller roller) {
		this.roller = roller;
	}

	//@formatter:off
	public Command setPower(double power) {
		return new FunctionalCommand(
				() -> {},
				() -> roller.setPower(power),
				interrupted -> roller.stop(),
				() -> false,
				roller
		).withName("Set power: " + power);
	}

	public Command setPower(DoubleSupplier power) {
		return new FunctionalCommand(
				() -> {},
				() -> roller.setPower(power.getAsDouble()),
				interrupted -> roller.stop(),
				() -> false,
				roller
		).withName("Set power by supplier");
	}


	public Command rollRotations(Rotation2d rotations, double power) {
		Rotation2d startingPosition = roller.getPosition();
		return new FunctionalCommand(
				()-> {},
				()-> roller.setPower(power),
				interrupted -> roller.stop(),
				() -> Math.abs(roller.getPosition().getRotations() - startingPosition.getRotations()) > rotations.getRotations(),
				roller
		).withName("Rotate rotations: " + rotations);
	}

	public Command stop() {
		return new InstantCommand(roller::stop, roller).withName("Stop");
	}
	//@formatter:on

}
