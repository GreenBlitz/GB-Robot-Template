package frc.robot.autonomous;

import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Robot;
import frc.utils.auto.PathPlannerUtils;

import java.util.Optional;

public class SequencesBuilder {

	public static Command Intake(Robot robot, String pathName, String logPathPrefix) {
		Optional<PathPlannerPath> pathOptional = PathPlannerUtils.getPathFromFile(pathName, logPathPrefix + "Intake/");
		return PathPlannerUtils.safelyApplyPathToCommandFunction(
			path -> new ParallelCommandGroup(PathPlannerUtils.followPathOrDriveToPathEnd(robot, path), AutonomousConstants.INTAKE_COMMAND.get()),
			pathOptional
		);
	}

	public static Command IntakeShoot(Robot robot, String pathName, String logPathPrefix) {
		return new SequentialCommandGroup(Intake(robot, pathName, logPathPrefix + "IntakeShoot/"), AutonomousConstants.SHOOTING_COMMAND.get());
	}

	public static Command Shooting(Robot robot, String pathName, String logPathPrefix) {
		Optional<PathPlannerPath> pathOptional = PathPlannerUtils.getPathFromFile(pathName, logPathPrefix + "Shoot/");
		return PathPlannerUtils.safelyApplyPathToCommandFunction(
			path -> new SequentialCommandGroup(
				new ParallelDeadlineGroup(
					PathPlannerUtils.followPathOrDriveToPathEnd(robot, path),
					AutonomousConstants.BEFORE_SHOOTING_COMMAND.get()
				),
				AutonomousConstants.SHOOTING_COMMAND.get()
			),
			pathOptional
		);
	}

	public static Command ShootOnMove(Robot robot, String pathName, String logPathPrefix) {
		Optional<PathPlannerPath> pathOptional = PathPlannerUtils.getPathFromFile(pathName, logPathPrefix + "ShootOnMove/");
		return PathPlannerUtils.safelyApplyPathToCommandFunction(
			path -> new ParallelCommandGroup(
				PathPlannerUtils.followPathOrDriveToPathEnd(robot, path),
				AutonomousConstants.SHOOTING_COMMAND.get()
			),
			pathOptional
		);
	}

}
