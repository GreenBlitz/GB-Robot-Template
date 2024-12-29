package frc.robot.autonomous;

import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Robot;
import frc.utils.auto.PathPlannerUtils;

import java.util.function.Supplier;

public class SequencesBuilder {

	public static Command Intake(Robot robot, PathPlannerPath path, Supplier<Command> intakeCommand) {
		return new ParallelCommandGroup(PathPlannerUtils.followPathOrDriveToPathEnd(robot, path), intakeCommand.get());
	}

	public static Command IntakeShoot(Robot robot, PathPlannerPath path, Supplier<Command> intakeCommand, Supplier<Command> shootingCommand) {
		return new SequentialCommandGroup(Intake(robot, path, intakeCommand), shootingCommand.get());
	}

	public static Command Shooting(Robot robot, PathPlannerPath path, Supplier<Command> preShootingCommand, Supplier<Command> shootingCommand) {
		return new SequentialCommandGroup(
			new ParallelDeadlineGroup(PathPlannerUtils.followPathOrDriveToPathEnd(robot, path), preShootingCommand.get()),
			shootingCommand.get()
		);
	}

	public static Command ShootOnMove(Robot robot, PathPlannerPath path, Supplier<Command> shootingCommand) {
		return new ParallelCommandGroup(PathPlannerUtils.followPathOrDriveToPathEnd(robot, path), shootingCommand.get());
	}

}
