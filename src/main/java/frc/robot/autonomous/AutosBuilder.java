package frc.robot.autonomous;

import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Robot;
import frc.utils.auto.AutoPath;
import frc.utils.auto.GBAuto;
import frc.utils.auto.PathPlannerUtils;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.function.Function;
import java.util.function.Supplier;

public class AutosBuilder {

	public static List<Supplier<GBAuto>> getAllPathPlannerAutos() {
		return List.of(() -> new GBAuto("Rotate"), () -> new GBAuto("Rotate 2m"), () -> new GBAuto("Straight 2m"));
	}

	public static List<Supplier<GBAuto>> getAllAutoLineAutos(Robot robot, Supplier<Command> scoringCommand) {
		ArrayList<Supplier<GBAuto>> autos = new ArrayList<>();
		for (AutoPath autoPath : AutoPath.getAllAutoLinePaths()) {
			autos.add(
				() -> createAutoFromAutoPath(
					autoPath,
					scoringCommand,
					pathPlannerPath -> SequencesBuilder.commandAfterPath(robot, pathPlannerPath, scoringCommand)
				)
			);
		}
		return autos;
	}

	public static List<Supplier<GBAuto>> getAllIntakeAutos(Robot robot, Supplier<Command> intakeCommand) {
		ArrayList<Supplier<GBAuto>> autos = new ArrayList<>();
		for (AutoPath autoPath : AutoPath.getAllPathsToCoralStations()) {
			autos.add(
				() -> createAutoFromAutoPath(
					autoPath,
					pathPlannerPath -> SequencesBuilder.commandAfterPath(robot, pathPlannerPath, intakeCommand)
				)
			);
		}
		return autos;
	}

	public static List<Supplier<GBAuto>> getAllScoringAutos(Robot robot, Supplier<Command> scoringCommand) {
		ArrayList<Supplier<GBAuto>> autos = new ArrayList<>();
		for (AutoPath autoPath : AutoPath.getAllPathsToCoralStations()) {
			autos.add(
				() -> createAutoFromAutoPath(
					autoPath,
					pathPlannerPath -> SequencesBuilder.commandAfterPath(robot, pathPlannerPath, scoringCommand)
				)
			);
		}
		return autos;
	}


	private static GBAuto createAutoFromAutoPath(AutoPath path, Function<PathPlannerPath, Command> pathFollowingCommand) {
		Optional<PathPlannerPath> pathOptional = path.getPath();

		return new GBAuto(
			pathOptional.map(pathFollowingCommand).orElse(Commands.none()),
			pathOptional.map(PathPlannerUtils::getPathStartingPose).orElse(path.getStartingPoint().getSecond()),
			path.getPathName(),
			pathOptional.isPresent()
		);
	}

}
