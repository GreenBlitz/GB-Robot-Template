package frc.robot.subsystems.algaeIntake.pivot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;

public class PivotCommandsBuilder {

	private final Pivot pivot;

	public PivotCommandsBuilder(Pivot pivot) {
		this.pivot = pivot;
	}

	public Command moveToPosition(Rotation2d targetPosition) {
		return pivot.asSubsystemCommand(
			new RunCommand(() -> pivot.setTargetPosition(targetPosition), pivot),
			"Set target position to: " + targetPosition
		);
	}

	public Command stayInPlace() {
		return pivot.asSubsystemCommand(new RunCommand(pivot::stayInPlace, pivot), "Stay in place");
	}

}
