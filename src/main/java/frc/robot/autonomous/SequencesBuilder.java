package frc.robot.autonomous;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Robot;
import frc.utils.auto.PathPlannerUtils;

public class SequencesBuilder {

	public static Command Intake(Robot robot, String pathName, String logPathPrefix) {
		return new ParallelCommandGroup(
			PathPlannerUtils.followPathOrDriveToPathEnd(robot, pathName, logPathPrefix + "Intake/"),
			AutonomousConstants.INTAKE_COMMAND.get()
		);
	}

	public static Command IntakeShoot(Robot robot, String pathName, String logPathPrefix) {
		return new SequentialCommandGroup(Intake(robot, pathName, logPathPrefix + "IntakeShoot/"), AutonomousConstants.SHOOTING_COMMAND.get());
	}

	public static Command Shooting(Robot robot, String pathName, String logPathPrefix) {
		return new SequentialCommandGroup(
			new ParallelDeadlineGroup(
				PathPlannerUtils.followPathOrDriveToPathEnd(robot, pathName, logPathPrefix + "Shoot/"),
				AutonomousConstants.BEFORE_SHOOTING_COMMAND.get()
			),
			AutonomousConstants.SHOOTING_COMMAND.get()
		);
	}

	public static Command ShootOnMove(Robot robot, String pathName, String logPathPrefix) {
		return new ParallelCommandGroup(
			PathPlannerUtils.followPathOrDriveToPathEnd(robot, pathName, logPathPrefix + "ShootOnMove/"),
			AutonomousConstants.SHOOTING_COMMAND.get()
		);
	}

}
