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

import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;

public class AutosBuilder {

	public static List<GBAuto> getAllAutos(Robot robot, Supplier<Command> intakeCommand, Supplier<Command> shootingCommand) {
		return List.of(M231(robot, intakeCommand, shootingCommand), new GBAuto("Rotate"), new GBAuto("Rotate 2m"), new GBAuto("Straight 2m"));
	}

	private static GBAuto M231(Robot robot, Supplier<Command> intakeCommand, Supplier<Command> shootingCommand) {
		String logPath = AutonomousConstants.LOG_PATH_PREFIX + "M231/";
		Optional<PathPlannerPath> pathM2 = AutoPath.MIDDLE_OF_SUBWOOFER_TO_NOTE_2.getPathOptional(logPath);
		Optional<PathPlannerPath> path23 = AutoPath.NOTE_2_TO_NOTE_3.getPathOptional(logPath);
		Optional<PathPlannerPath> path31 = AutoPath.NOTE_3_TO_NOTE_1.getPathOptional(logPath);
		Pose2d startingPoint = pathM2.map(PathPlannerUtils::getPathStartingPose).orElse(Pose2d.kZero);

		return new GBAuto(
			new SequentialCommandGroup(
				shootingCommand.get(),
				pathM2.map(path -> SequencesBuilder.IntakeShoot(robot, path, intakeCommand, shootingCommand)).orElseGet(Commands::none),
				path23.map(path -> SequencesBuilder.IntakeShoot(robot, path, intakeCommand, shootingCommand)).orElseGet(Commands::none),
				path31.map(path -> SequencesBuilder.IntakeShoot(robot, path, intakeCommand, shootingCommand)).orElseGet(Commands::none)
			),
			startingPoint,
			"M231",
			pathM2.isPresent() && path23.isPresent() && path31.isPresent()
		);
	}

}
