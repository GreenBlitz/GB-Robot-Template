package frc.robot.subsystems.intake.pivot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class PivotCommandBuilder {

	private final Pivot pivot;

	public PivotCommandBuilder(Pivot pivot) {
		this.pivot = pivot;
	}

	public Command goToPosition(Rotation2d position) {
		return new FunctionalCommand(
			() -> {},
			() -> pivot.setPosition(position),
			interrupted -> pivot.stop(),
			() -> pivot.isAtAngle(position),
			pivot
		).withName("Go to position: " + position.getDegrees());
	}

	public Command goToPosition(double position) {
		return new FunctionalCommand(
			() -> {},
			() -> pivot.setPosition(Rotation2d.fromDegrees(position)),
			interrupted -> pivot.stop(),
			() -> pivot.isAtAngle(Rotation2d.fromDegrees(position)),
			pivot
		).withName("Go to position: " + position);
	}

	public Command setPower(double power) {
		return new InstantCommand(() -> pivot.setPower(power)).withName("Set power to: " + power);
	}

	public Command stop() {
		return new InstantCommand(pivot::stop, pivot).withName("Stop");
	}

	public Command stayInPlace() {
		return new InstantCommand(pivot::stayInPlace, pivot).withName("Stay in place");
	}

}
