package frc.robot.subsystems.elbow;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.pivot.PivotState;

public class ElbowStateHandler {

    private static final Rotation2d TOLERANCE = Rotation2d.fromDegrees(2);

    private final Elbow elbow;

    public ElbowStateHandler(Elbow elbow) {
        this.elbow = elbow;
    }

    public Command setState(ElbowState elbowState) {
        return elbow.getCommandsBuilder().moveToAngle(elbowState.getTargetPosition(), TOLERANCE);
    }

}
