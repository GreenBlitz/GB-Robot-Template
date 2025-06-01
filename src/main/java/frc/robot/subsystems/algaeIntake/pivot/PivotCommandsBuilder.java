package frc.robot.subsystems.algaeIntake.pivot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;

public class PivotCommandsBuilder {

	private final Pivot pivot;

	public PivotCommandsBuilder(Pivot pivot) {
		this.pivot = pivot;
	}

	public Command moveToPosition(Rotation2d targetPosition){
		return new RunCommand(() -> pivot.setTargetPosition(targetPosition), pivot);
	}

	public Command stayInPlace(){
		return new RunCommand(pivot::stayInPlace, pivot);
	}

}
