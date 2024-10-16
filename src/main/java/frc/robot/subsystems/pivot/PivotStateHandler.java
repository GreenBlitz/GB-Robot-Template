package frc.robot.subsystems.pivot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.constants.Field;

import java.util.Optional;
import java.util.function.Supplier;

public class PivotStateHandler {

	private final Pivot pivot;

	private final Optional<Supplier<Pose2d>> robotPoseSupplier;

	public PivotStateHandler(Pivot pivot, Optional<Supplier<Pose2d>> robotPoseSupplier) {
		this.pivot = pivot;
		this.robotPoseSupplier = robotPoseSupplier;
	}

	public Command setState(PivotState pivotState) {
		if (pivotState == PivotState.INTERPOLATE) {
			if (robotPoseSupplier.isEmpty()) {
				Command emptyCommand = new InstantCommand();
				emptyCommand.addRequirements(pivot);
				return emptyCommand;
			}
			return pivot.getCommandsBuilder()
				.moveToPosition(
					() -> Rotation2d
						.fromRadians(PivotInterpolationMap.METERS_TO_RADIANS.get(Field.getMetersFromSpeaker(robotPoseSupplier.get().get())))
				);
		}
		return pivot.getCommandsBuilder().moveToPosition(pivotState.getTargetPosition());
	}

}
