package frc.robot.subsystems.intake.pivot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.utils.utilcommands.LoggedDashboardCommand;

public class PivotCommandsBuilder {

	private final Pivot pivot;

	public PivotCommandsBuilder(Pivot pivot) {
		this.pivot = pivot;
	}

	//@formatter:off
	public Command goToPosition(Rotation2d position) {
		return new FunctionalCommand(
			() -> {},
			() -> pivot.setTargetPosition(position),
			interrupted -> pivot.stayInPlace(),
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

	public Command calibrateKg() {
		return new LoggedDashboardCommand("pivot kg calibration", pivot::setVoltage, pivot);
	}

	public Command stop() {
		return new RunCommand(pivot::stop, pivot).withName("Stop");
	}
	//@formatter:on

}
