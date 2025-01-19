package frc.robot.autonomous;

import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Robot;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

public class SequencesBuilder {

	public static Command commandDuringPath(Robot robot, PathPlannerPath path, Supplier<Command> commandSupplier) {
		return new ParallelCommandGroup(RobotAutoHelper.followPathOrDriveToPathEnd(robot, path), commandSupplier.get());
	}

	public static Command commandWhenConditionIsMetDuringPath(
		Robot robot,
		PathPlannerPath path,
		Supplier<Command> commandSupplier,
		BooleanSupplier condition
	) {
		return commandDuringPath(robot, path, () -> new WaitUntilCommand(condition).andThen(commandSupplier.get()));
	}

	public static Command commandAfterPath(Robot robot, PathPlannerPath path, Supplier<Command> commandSupplier) {
		return new SequentialCommandGroup(RobotAutoHelper.followPathOrDriveToPathEnd(robot, path), commandSupplier.get());
	}

	public static Command feedAndScore(
		Robot robot,
		PathPlannerPath pathToSource,
		PathPlannerPath pathFromSource,
		Supplier<Command> feedingCommand,
		Supplier<Command> scoringCommand,
		BooleanSupplier feedingStartCondition,
		BooleanSupplier scoringStartCondition
	) {
		return new SequentialCommandGroup(
			commandWhenConditionIsMetDuringPath(robot, pathToSource, feedingCommand, feedingStartCondition),
			commandWhenConditionIsMetDuringPath(robot, pathFromSource, scoringCommand, scoringStartCondition)
		);
	}

}
