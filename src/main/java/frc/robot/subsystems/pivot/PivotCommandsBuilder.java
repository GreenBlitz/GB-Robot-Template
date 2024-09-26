package frc.robot.subsystems.pivot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.pivot.factories.PivotFactory;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class PivotCommandsBuilder {

	private final Pivot pivot;

	public PivotCommandsBuilder(Pivot pivot) {
		this.pivot = pivot;
	}

	public Command setPower(double power) {
		return new FunctionalCommand(() -> {}, () -> pivot.setPower(power), (interrupted) -> pivot.stop(), () -> false, pivot)
			.withName("Set Power to: " + power);
	}

	public Command setPower(DoubleSupplier power) {
		return new FunctionalCommand(() -> {}, () -> pivot.setPower(power.getAsDouble()), (interrupted) -> pivot.stop(), () -> false, pivot)
			.withName("Set Supplied Power");
	}

	public Command moveToPosition(Rotation2d position) {
		return new FunctionalCommand(
			() -> {},
			() -> pivot.setTargetPosition(position),
			(interrupted) -> {},
			() -> pivot.isAtPosition(position, PivotFactory.getTolerance()),
			pivot
		).withName("Move to: " + position);
	}

	public Command moveToPosition(Supplier<Rotation2d> positionSupplier) {
		return new FunctionalCommand(
			() -> {},
			() -> pivot.setTargetPosition(positionSupplier.get()),
			(interrupted) -> pivot.stop(),
			() -> false,
			pivot
		).withName("Move to Supplier Position");
	}

	public Command stop() {
		return new InstantCommand(pivot::stop, pivot).withName("Stop");
	}

}
