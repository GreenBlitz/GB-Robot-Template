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
	public Command moveByPower(double power) {
		return new FunctionalCommand(
				() -> {},
				() -> roller.setPower(power),
				interrupted -> roller.stop(),
				() -> false,
				roller
		).withName("Move by power: " + power);
	}

	public Command moveByPower(DoubleSupplier power) {
		return new FunctionalCommand(
				() -> {},
				() -> roller.setPower(power.getAsDouble()),
				interrupted -> roller.stop(),
				() -> false, roller
		).withName("Move by power supplier");
	}

	public Command rollRotations(Rotation2d rotationsToAdd, double power) {
		return new FunctionalCommand(
				()-> roller.setTargetPositionWithAddition(rotationsToAdd),
				()-> roller.setPower(power),
				interrupted -> roller.stop(),
				roller::isPastTargetPosition
		).withName("Roll rotations: " + rotationsToAdd);
	}
	//@formatter:on

	public Command stop() {
		return new InstantCommand(roller::stop, roller).withName("Stop");
	}

}
