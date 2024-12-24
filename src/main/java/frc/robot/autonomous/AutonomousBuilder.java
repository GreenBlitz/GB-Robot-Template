package frc.robot.autonomous;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Robot;
import frc.utils.auto.PathPlannerUtils;

import java.util.HashMap;
import java.util.Map;
import java.util.Optional;

public class AutonomousBuilder {

	public static Map<String, Command> getAllAutos(Robot robot) {
		Map<String, Command> autoMap = new HashMap<>();
		autoMap.put("M231", withResetOdometry(M231(robot)));
		autoMap.put("Rotate", new PathPlannerAuto("Rotate"));
		autoMap.put("Rotate 2m", new PathPlannerAuto("Rotate 2m"));
		autoMap.put("Straight 2m", new PathPlannerAuto("Straight 2m"));
		return autoMap;
	}

	private static Command withResetOdometry(PathPlannerAuto auto) {
		return AutoBuilder.resetOdom(auto.getStartingPose()).andThen(auto);
	}

	private static PathPlannerAuto M231(Robot robot) {
		String logPath = AutonomousConstants.LOG_PATH_PREFIX + "M231/";
		Optional<PathPlannerPath> pathM2 = PathPlannerUtils.getPathFromFile("M2", logPath);
		Optional<PathPlannerPath> path23 = PathPlannerUtils.getPathFromFile("23", logPath);
		Optional<PathPlannerPath> path31 = PathPlannerUtils.getPathFromFile("31", logPath);
		Pose2d startingPoint = pathM2.map(PathPlannerUtils::getPathStartingPose).orElse(Pose2d.kZero);

		return new PathPlannerAuto(
			new SequentialCommandGroup(
				AutonomousConstants.SHOOTING_COMMAND.apply(robot),
				pathM2.map(path -> SequencesBuilder.IntakeShoot(robot, path)).orElseGet(Commands::none),
				path23.map(path -> SequencesBuilder.IntakeShoot(robot, path)).orElseGet(Commands::none),
				path31.map(path -> SequencesBuilder.IntakeShoot(robot, path)).orElseGet(Commands::none)
			),
			startingPoint
		);
	}

}
