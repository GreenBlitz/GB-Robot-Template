package frc.robot.autonomous;

import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Robot;
import frc.utils.auto.AutoPath;
import frc.utils.auto.GBAuto;
import frc.utils.auto.PathPlannerUtils;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.Function;
import java.util.function.Supplier;

public class AutosBuilder {

	public static List<Supplier<GBAuto>> getAllPathPlannerAutos() {
		return List.of(() -> new GBAuto("Rotate"), () -> new GBAuto("Rotate 2m"), () -> new GBAuto("Straight 2m"));
	}

	public static List<Supplier<GBAuto>> getAllAutoLineAutos(
		Robot robot,
		Supplier<Command> scoringCommand,
		Function<Translation2d, Boolean> scoringStartCondition
	) {
		ArrayList<Supplier<GBAuto>> autos = new ArrayList<>();
		for (AutoPath autoPath : AutoPath.getAllAutoLinePaths()) {
			autos.add(
				() -> commandWhenConditionIsMetDuringPath(
					robot,
					autoPath,
					scoringCommand,
					() -> scoringStartCondition.apply(autoPath.getEndPoint().getSecond())
				)
			);
		}
		return autos;
	}

	public static List<Supplier<GBAuto>> getAllFeedScoreSequences(
		Robot robot,
		Supplier<Command> feedingCommand,
		Supplier<Command> scoringCommand,
		Function<Translation2d, Boolean> feedingStartCondition,
		Function<Translation2d, Boolean> scoringStartCondition
	) {
		ArrayList<Supplier<GBAuto>> autos = new ArrayList<>();
		for (AutoPath pathToCoralStation : AutoPath.getAllPathsToCoralStations()) {
			for (AutoPath pathFromCoralStation : AutoPath.getAllPathsFromCoralStations()) {
				autos.add(
					() -> feedScoreSequence(
						robot,
						pathToCoralStation,
						pathFromCoralStation,
						feedingCommand,
						scoringCommand,
						feedingStartCondition,
						scoringStartCondition
					)
				);
			}
		}
		return autos;
	}

	private static GBAuto feedScoreSequence(
		Robot robot,
		AutoPath pathToCoralStation,
		AutoPath pathFromCoralStation,
		Supplier<Command> feedingCommand,
		Supplier<Command> scoringCommand,
		Function<Translation2d, Boolean> feedingStartCondition,
		Function<Translation2d, Boolean> scoringStartCondition
	) {
		GBAuto feedingAuto = commandWhenConditionIsMetDuringPath(
			robot,
			pathToCoralStation,
			feedingCommand,
			() -> feedingStartCondition.apply(pathToCoralStation.getEndPoint().getSecond())
		);
		GBAuto scoringAuto = commandWhenConditionIsMetDuringPath(
			robot,
			pathFromCoralStation,
			scoringCommand,
			() -> scoringStartCondition.apply(pathFromCoralStation.getEndPoint().getSecond())
		);

		return new GBAuto(
			feedingAuto.andThen(scoringAuto).onlyIf(() -> feedingAuto.isFullyCreated() && scoringAuto.isFullyCreated()),
			feedingAuto.getStartingPose(),
			pathToCoralStation.getPathName() + "-" + pathFromCoralStation.getEndPoint().getFirst(),
			feedingAuto.isFullyCreated() && scoringAuto.isFullyCreated()
		);
	}

	private static GBAuto commandWhenConditionIsMetDuringPath(
		Robot robot,
		AutoPath path,
		Supplier<Command> commandSupplier,
		BooleanSupplier condition
	) {
		Optional<PathPlannerPath> pathOptional = path.getPath();

		return new GBAuto(
			pathOptional
				.map(pathPlannerPath -> SequencesBuilder.commandWhenConditionIsMetDuringPath(robot, pathPlannerPath, commandSupplier, condition))
				.orElse(Commands.none()),
			pathOptional.map(PathPlannerUtils::getPathStartingPose).orElse(Pose2d.kZero),
			path.getPathName(),
			pathOptional.isPresent()
		);
	}

}
