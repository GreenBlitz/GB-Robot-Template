package frc.robot.autonomous;

import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Robot;
import frc.utils.auto.PathPlannerUtils;


public class SequencesBuilder {

	public static Command Intake(Robot robot, PathPlannerPath path) {
		return new ParallelCommandGroup(
			PathPlannerUtils.followPathOrDriveToPathEnd(robot, path),
			AutonomousConstants.INTAKE_COMMAND.apply(robot)
		);
	}

	public static Command IntakeShoot(Robot robot, PathPlannerPath path) {
		return new SequentialCommandGroup(Intake(robot, path), AutonomousConstants.SHOOTING_COMMAND.apply(robot));
	}

	public static Command Shooting(Robot robot, PathPlannerPath path) {
		return new SequentialCommandGroup(
			new ParallelDeadlineGroup(
				PathPlannerUtils.followPathOrDriveToPathEnd(robot, path),
				AutonomousConstants.BEFORE_SHOOTING_COMMAND.apply(robot)
			),
			AutonomousConstants.SHOOTING_COMMAND.apply(robot)
		);
	}

	public static Command ShootOnMove(Robot robot, PathPlannerPath path) {
		return new ParallelCommandGroup(
			PathPlannerUtils.followPathOrDriveToPathEnd(robot, path),
			AutonomousConstants.SHOOTING_COMMAND.apply(robot)
		);
	}

}
