package frc.robot.subsystems.pivot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.utils.utilcommands.LoggedDashboardCommand;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class PivotCommandsBuilder {

	private final Pivot pivot;

	public PivotCommandsBuilder(Pivot pivot) {
		this.pivot = pivot;
	}

	//@formatter:off
	public Command setPower(double power) {
		return new FunctionalCommand(
			() -> {},
			() -> pivot.setPower(power),
			interrupted -> pivot.stop(),
			() -> false,
			pivot
		).withName("Set power to: " + power);
	}

	public Command setPower(DoubleSupplier power) {
		return new FunctionalCommand(
			() -> {},
			() -> pivot.setPower(power.getAsDouble()),
			interrupted -> pivot.stop(),
			() -> false,
			pivot
		).withName("Set power by supplier");
	}

	public Command moveToPosition(Rotation2d position) {
		return new FunctionalCommand(
			() -> pivot.setTargetPosition(position),
			() -> {},
			interrupted -> pivot.stayInPlace(),
			() -> false,
			pivot
		).withName("Move to position: " + position);
	}

	public Command moveToPosition(Supplier<Rotation2d> positionSupplier) {
		return new FunctionalCommand(
			() -> {},
			() -> pivot.setTargetPosition(positionSupplier.get()),
			interrupted -> pivot.stayInPlace(),
			() -> false,
			pivot
		).withName("Move to supplier position");
	}

	public Command calibInterpolation() {
		return new LoggedDashboardCommand(
			"calibInterpolation",
			position -> pivot.setTargetPosition(Rotation2d.fromDegrees(position)),
			pivot
		);
	}

	public Command useInterpolation() {
		return new LoggedDashboardCommand(
			"use Interpolation",
			distance -> pivot.setTargetPosition(Rotation2d.fromRadians(PivotInterpolationMap.METERS_TO_RADIANS.get(distance))),
			pivot
		);
	}

	public Command stop() {
		return new RunCommand(pivot::stop, pivot).withName("Stop");
	}
	//@formatter:on

}
