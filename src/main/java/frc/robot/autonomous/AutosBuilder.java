package frc.robot.autonomous;

import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Robot;
import frc.utils.auto.AutoPath;
import frc.utils.auto.GBAuto;
import frc.utils.auto.PathPlannerUtils;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;

public class AutosBuilder {

	public static List<Supplier<GBAuto>> getAllAutos(Robot robot, Supplier<Command> intakeCommand, Supplier<Command> shootingCommand) {
		ArrayList<Supplier<GBAuto>> autoList = new ArrayList<>(getAllGUIAutos());
		autoList.add(() -> M231(robot, intakeCommand, shootingCommand));
		return autoList;
	}

	public static List<Supplier<GBAuto>> getAllGUIAutos() {
		return List.of(() -> new GBAuto("Rotate"), () -> new GBAuto("Rotate 2m"), () -> new GBAuto("Straight 2m"));
	}

	public static List<Supplier<GBAuto>> getAllAutoLineAutos(Robot robot, Supplier<Command> scoringCommand) {
		Optional<PathPlannerPath> pathAutoLine2ToI = AutoPath.AUTO_LINE_2_TO_I.getPathOptional();
		Optional<PathPlannerPath> pathAutoLine4ToH = AutoPath.AUTO_LINE_4_TO_H.getPathOptional();
		Optional<PathPlannerPath> pathAutoLine6ToF = AutoPath.AUTO_LINE_6_TO_F.getPathOptional();

		return List.of(
			() -> new GBAuto(
				pathAutoLine2ToI.map(path -> SequencesBuilder.commandAfterPath(robot, path, scoringCommand)).orElseGet(Commands::none),
				pathAutoLine2ToI.map(PathPlannerUtils::getPathStartingPose).orElse(Pose2d.kZero),
				AutoPath.AUTO_LINE_2_TO_I.getPathName(),
				pathAutoLine2ToI.isPresent()
			),
			() -> new GBAuto(
				pathAutoLine4ToH.map(path -> SequencesBuilder.commandAfterPath(robot, path, scoringCommand)).orElseGet(Commands::none),
				pathAutoLine4ToH.map(PathPlannerUtils::getPathStartingPose).orElse(Pose2d.kZero),
				AutoPath.AUTO_LINE_4_TO_H.getPathName(),
				pathAutoLine4ToH.isPresent()
			),
			() -> new GBAuto(
				pathAutoLine6ToF.map(path -> SequencesBuilder.commandAfterPath(robot, path, scoringCommand)).orElseGet(Commands::none),
				pathAutoLine6ToF.map(PathPlannerUtils::getPathStartingPose).orElse(Pose2d.kZero),
				AutoPath.AUTO_LINE_6_TO_F.getPathName(),
				pathAutoLine6ToF.isPresent()
			)
		);
	}

	private static GBAuto M231(Robot robot, Supplier<Command> intakeCommand, Supplier<Command> shootingCommand) {
		Optional<PathPlannerPath> pathM2 = AutoPath.MIDDLE_OF_SUBWOOFER_TO_NOTE_2.getPathOptional();
		Optional<PathPlannerPath> path23 = AutoPath.NOTE_2_TO_NOTE_3.getPathOptional();
		Optional<PathPlannerPath> path31 = AutoPath.NOTE_3_TO_NOTE_1.getPathOptional();
		Pose2d startingPoint = pathM2.map(PathPlannerUtils::getPathStartingPose).orElse(Pose2d.kZero);

		return new GBAuto(
			new SequentialCommandGroup(
				shootingCommand.get(),
				pathM2.map(path -> SequencesBuilder.intakeShoot(robot, path, intakeCommand, shootingCommand)).orElseGet(Commands::none),
				path23.map(path -> SequencesBuilder.intakeShoot(robot, path, intakeCommand, shootingCommand)).orElseGet(Commands::none),
				path31.map(path -> SequencesBuilder.intakeShoot(robot, path, intakeCommand, shootingCommand)).orElseGet(Commands::none)
			),
			startingPoint,
			"M231",
			pathM2.isPresent() && path23.isPresent() && path31.isPresent()
		);
	}

}
