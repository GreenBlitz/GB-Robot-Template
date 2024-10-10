package frc.robot.subsystems.pivot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.constants.Field;

import java.util.function.Supplier;

public class PivotStateHandler {

	private final Pivot pivot;

	public PivotStateHandler(Pivot pivot) {
		this.pivot = pivot;
    }

	public Command setState(PivotState pivotState) {
		return pivot.getCommandsBuilder().moveToPosition(pivotState.getTargetPosition());
	}

}
