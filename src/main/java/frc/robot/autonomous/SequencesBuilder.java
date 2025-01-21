package frc.robot.autonomous;

import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Robot;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

public class SequencesBuilder {

	public static Command commandDuringPath(Robot robot, PathPlannerPath path, Supplier<Command> commandSupplier) {
		return new ParallelDeadlineGroup(commandSupplier.get(), PathFollowingCommands.followPathOrDriveToPathEnd(robot, path));
	}

	public static Command commandWhenConditionIsMetDuringPath(
		Robot robot,
		PathPlannerPath path,
		Supplier<Command> commandSupplier,
		BooleanSupplier condition
	) {
		return commandDuringPath(robot, path, () -> new WaitUntilCommand(condition).andThen(commandSupplier.get()));
	}

}
