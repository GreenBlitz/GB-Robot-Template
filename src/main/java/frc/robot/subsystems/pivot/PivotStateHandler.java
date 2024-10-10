package frc.robot.subsystems.pivot;

import edu.wpi.first.wpilibj2.command.Command;


public class PivotStateHandler {

	private final Pivot pivot;

	public PivotStateHandler(Pivot pivot) {
		this.pivot = pivot;
	}

	public Command setState(PivotState pivotState) {
		return pivot.getCommandsBuilder().moveToPosition(pivotState.getTargetPosition());
	}

}
