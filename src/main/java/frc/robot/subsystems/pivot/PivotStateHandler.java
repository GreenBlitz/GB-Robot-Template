package frc.robot.subsystems.pivot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;

public class PivotStateHandler {

    private static final Rotation2d TOLERANCE = Rotation2d.fromDegrees(1);

    private final Pivot pivot;

    public PivotStateHandler(Pivot pivot) {
        this.pivot = pivot;
    }

    public Command setState(PivotState pivotState) {
        return pivot.getCommandsBuilder().moveToPosition(pivotState.getTargetPosition(), TOLERANCE);
    }

}
