package frc.robot.subsystems.intake.pivot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;

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
			() -> false,
			pivot
		).withName("Go to position: " + position.getDegrees());
	}

	public Command setPower(double power) {
		return new FunctionalCommand(
				() -> {},
				() -> pivot.setPower(power),
				interrupted -> pivot.stop(),
				() -> false,
				pivot
		).withName("Set power to: " + power);
	}

	public Command stop() {
		return new RunCommand(pivot::stop, pivot).withName("Stop");
	}

}
