package frc.robot.subsystems.pivot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.constants.Field;

import java.util.function.Supplier;

public class PivotStateHandler {

	private final Pivot pivot;
	private final Supplier<Pose2d> robotPoseSupplier;

	public PivotStateHandler(Pivot pivot, Supplier<Pose2d> robotPoseSupplier) {
		this.pivot = pivot;
        this.robotPoseSupplier = robotPoseSupplier;
    }

	public Command setState(PivotState pivotState) {
		if(pivotState == PivotState.INTERPOLATE){
			if(robotPoseSupplier == null){
				return new InstantCommand(() -> {}, pivot);
			}
			return pivot.getCommandsBuilder().moveToPosition(
					() -> DistanceToAngleMap.getAngle(
							getDistanceFromRootToShooter(robotPoseSupplier.get())
					)
			);
		}
		return pivot.getCommandsBuilder().moveToPosition(pivotState.getTargetPosition());
	}

	private double getDistanceFromRootToShooter(Pose2d rootPose){
		return Field.getSpeaker().toTranslation2d().getDistance(rootPose.getTranslation());
	}

}
