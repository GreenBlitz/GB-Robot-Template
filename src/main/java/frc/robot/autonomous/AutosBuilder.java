package frc.robot.autonomous;

import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Robot;
import frc.utils.auto.AutoPath;
import frc.utils.auto.GBAuto;
import frc.utils.auto.PathPlannerUtils;

import java.util.List;
import java.util.Optional;
import java.util.function.Function;
import java.util.function.Supplier;

public class AutosBuilder {

	public static List<Supplier<GBAuto>> getAllGUIAutos() {
		return List.of(() -> new GBAuto("Rotate"), () -> new GBAuto("Rotate 2m"), () -> new GBAuto("Straight 2m"));
	}

	public static List<Supplier<GBAuto>> getAllAutoLineAutos(Robot robot, Supplier<Command> scoringCommand) {
		return List.of(() -> AL2I(robot, scoringCommand), () -> AL4H(robot, scoringCommand), () -> AL6F(robot, scoringCommand));
	}

	public static List<Supplier<GBAuto>> getAllFeedScoreSequences(
		Robot robot,
		Supplier<Command> feedingCommand,
		Supplier<Command> scoringCommand,
		Function<Pose2d, Boolean> feedingStartCondition,
		Function<Pose2d, Boolean> scoringStartCondition
	) {
		return List.of(() -> US_L(robot, feedingCommand, scoringCommand, feedingStartCondition, scoringStartCondition), () -> LS_C(robot, feedingCommand, scoringCommand, feedingStartCondition, scoringStartCondition));
	}

	private static GBAuto AL2I(Robot robot, Supplier<Command> scoringCommand) {
		Optional<PathPlannerPath> pathAutoLine2ToI = AutoPath.AUTO_LINE_2_TO_I.getPathOptional();
		return new GBAuto(
			pathAutoLine2ToI.map(path -> SequencesBuilder.commandAfterPath(robot, path, scoringCommand)).orElseGet(Commands::none),
			pathAutoLine2ToI.map(PathPlannerUtils::getPathStartingPose).orElse(Pose2d.kZero),
			AutoPath.AUTO_LINE_2_TO_I.getPathName(),
			pathAutoLine2ToI.isPresent()
		);
	}

	private static GBAuto AL4H(Robot robot, Supplier<Command> scoringCommand) {
		Optional<PathPlannerPath> pathAutoLine4ToH = AutoPath.AUTO_LINE_4_TO_H.getPathOptional();
		return new GBAuto(
			pathAutoLine4ToH.map(path -> SequencesBuilder.commandAfterPath(robot, path, scoringCommand)).orElseGet(Commands::none),
			pathAutoLine4ToH.map(PathPlannerUtils::getPathStartingPose).orElse(Pose2d.kZero),
			AutoPath.AUTO_LINE_4_TO_H.getPathName(),
			pathAutoLine4ToH.isPresent()
		);
	}

	private static GBAuto AL6F(Robot robot, Supplier<Command> scoringCommand) {
		Optional<PathPlannerPath> pathAutoLine6ToF = AutoPath.AUTO_LINE_6_TO_F.getPathOptional();
		return new GBAuto(
			pathAutoLine6ToF.map(path -> SequencesBuilder.commandAfterPath(robot, path, scoringCommand)).orElseGet(Commands::none),
			pathAutoLine6ToF.map(PathPlannerUtils::getPathStartingPose).orElse(Pose2d.kZero),
			AutoPath.AUTO_LINE_6_TO_F.getPathName(),
			pathAutoLine6ToF.isPresent()
		);
	}

	private static GBAuto US_L(Robot robot, Supplier<Command> feedingCommand, Supplier<Command> scoringCommand, Function<Pose2d, Boolean> feedingStartCondition, Function<Pose2d, Boolean> scoringStartCondition) {
		Optional<PathPlannerPath> pathIToUpperCoralStation = AutoPath.I_TO_UPPER_CORAL_STATION.getPathOptional();
		Optional<PathPlannerPath> pathUpperCoralStationToL = AutoPath.UPPER_CORAL_STATION_TO_L.getPathOptional();

		return new GBAuto(
			pathIToUpperCoralStation
				.map(
					pathToCoralStation -> pathUpperCoralStationToL
						.map(
							pathFromCoralStation -> SequencesBuilder
								.feedAndScore(robot, pathToCoralStation, pathFromCoralStation, feedingCommand, scoringCommand, () -> feedingStartCondition.apply(PathPlannerUtils.getLastPathPose(pathToCoralStation)), () -> scoringStartCondition.apply(PathPlannerUtils.getLastPathPose(pathFromCoralStation)))
						)
						.orElseGet(Commands::none)
				)
				.orElseGet(Commands::none),
			pathIToUpperCoralStation.map(PathPlannerUtils::getPathStartingPose).orElse(Pose2d.kZero),
			"US-L",
			pathIToUpperCoralStation.isPresent() && pathUpperCoralStationToL.isPresent()
		);
	}

	private static GBAuto LS_C(Robot robot, Supplier<Command> feedingCommand, Supplier<Command> scoringCommand, Function<Pose2d, Boolean> feedingStartCondition, Function<Pose2d, Boolean> scoringStartCondition) {
		Optional<PathPlannerPath> pathFToLowerCoralStation = AutoPath.F_TO_LOWER_CORAL_STATION.getPathOptional();
		Optional<PathPlannerPath> pathLowerCoralStationToC = AutoPath.LOWER_CORAL_STATION_TO_C.getPathOptional();

		return new GBAuto(
			pathFToLowerCoralStation
				.map(
					pathToCoralStation -> pathLowerCoralStationToC
						.map(
							pathFromCoralStation -> SequencesBuilder
								.feedAndScore(robot, pathToCoralStation, pathFromCoralStation, feedingCommand, scoringCommand, () -> feedingStartCondition.apply(PathPlannerUtils.getLastPathPose(pathToCoralStation)), () -> scoringStartCondition.apply(PathPlannerUtils.getLastPathPose(pathFromCoralStation)))
						)
						.orElseGet(Commands::none)
				)
				.orElseGet(Commands::none),
			pathFToLowerCoralStation.map(PathPlannerUtils::getPathStartingPose).orElse(Pose2d.kZero),
			"LS-C",
			pathFToLowerCoralStation.isPresent() && pathLowerCoralStationToC.isPresent()
		);
	}

}
