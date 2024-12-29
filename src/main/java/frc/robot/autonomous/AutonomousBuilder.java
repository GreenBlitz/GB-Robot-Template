package frc.robot.autonomous;

import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Robot;
import frc.utils.auto.PathPlannerUtils;

import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;

public class AutonomousBuilder {

	public static List<PathPlannerAuto> getAllAutos(Robot robot, Supplier<Command> intakeCommand, Supplier<Command> shootingCommand) {
		return List.of(
			M231(robot, intakeCommand, shootingCommand),
			new PathPlannerAuto("Rotate"),
			new PathPlannerAuto("Rotate 2m"),
			new PathPlannerAuto("Straight 2m")
		);
	}

	private static PathPlannerAuto M231(Robot robot, Supplier<Command> intakeCommand, Supplier<Command> shootingCommand) {
		String logPath = AutonomousConstants.LOG_PATH_PREFIX + "M231/";
		Optional<PathPlannerPath> pathM2 = PathPlannerUtils.getPathFromFile("M2", logPath);
		Optional<PathPlannerPath> path23 = PathPlannerUtils.getPathFromFile("23", logPath);
		Optional<PathPlannerPath> path31 = PathPlannerUtils.getPathFromFile("31", logPath);
		Pose2d startingPoint = pathM2.map(PathPlannerUtils::getPathStartingPose).orElse(Pose2d.kZero);

		PathPlannerAuto auto = new PathPlannerAuto(
			new SequentialCommandGroup(
				shootingCommand.get(),
				pathM2.map(path -> SequencesBuilder.IntakeShoot(robot, path, intakeCommand, shootingCommand)).orElseGet(Commands::none),
				path23.map(path -> SequencesBuilder.IntakeShoot(robot, path, intakeCommand, shootingCommand)).orElseGet(Commands::none),
				path31.map(path -> SequencesBuilder.IntakeShoot(robot, path, intakeCommand, shootingCommand)).orElseGet(Commands::none)
			),
			startingPoint
		);

		auto.setName("M231");
		if (pathM2.isEmpty() || path23.isEmpty() || path31.isEmpty()) {
			auto.setName(auto.getName() + " (partial)");
		}

		return auto;
	}

}
