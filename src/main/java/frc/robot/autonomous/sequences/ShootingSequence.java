package frc.robot.autonomous.sequences;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Robot;
import frc.robot.autonomous.AutonomousConstants;
import frc.utils.auto.PathPlannerUtils;

public class ShootingSequence extends SequentialCommandGroup {

	public ShootingSequence(Robot robot, String pathName) {
		super(
			new ParallelDeadlineGroup(
				PathPlannerUtils.followPathOrDriveToPathEnd(robot, pathName),
				AutonomousConstants.BEFORE_SHOOTING_COMMAND.get()
			),
			AutonomousConstants.SHOOTING_COMMAND.get()
		);
	}

}
