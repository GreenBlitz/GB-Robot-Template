package frc.robot.autonomous.sequences;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Robot;
import frc.robot.autonomous.AutonomousConstants;

public class ShootingSequence extends SequentialCommandGroup {

	public ShootingSequence(Robot robot, String pathName) {
		super(
			new ParallelCommandGroup(
				AutonomousConstants.PATHFIND_OR_FOLLOW_PATH_FUNCTION.apply(robot, pathName),
				AutonomousConstants.BEFORE_SHOOTING_COMMAND
			),
			AutonomousConstants.SHOOTING_COMMAND
		);
	}

}
