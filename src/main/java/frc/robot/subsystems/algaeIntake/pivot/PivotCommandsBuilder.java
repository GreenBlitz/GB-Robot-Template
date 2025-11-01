package frc.robot.subsystems.algaeIntake.pivot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.subsystems.GBCommandsBuilder;

public class PivotCommandsBuilder extends GBCommandsBuilder {

	private final Pivot pivot;

	public PivotCommandsBuilder(Pivot pivot) {
		super();
		this.pivot = pivot;
	}

	public Command moveToPosition(Rotation2d targetPosition) {
		return pivot
			.asSubsystemCommand(new RunCommand(() -> pivot.setTargetPosition(targetPosition)), "Set target position to: " + targetPosition);
	}

	public Command stayInPlace() {
		return pivot.asSubsystemCommand(new RunCommand(pivot::stayInPlace), "Stay in place");
	}

	public Command setPower(double power) {
		return pivot.asSubsystemCommand(new RunCommand(() -> pivot.setPower(power)), "set power to " + power);
	}

}
