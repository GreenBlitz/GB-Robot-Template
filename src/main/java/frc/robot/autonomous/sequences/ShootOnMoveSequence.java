package frc.robot.autonomous.sequences;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Robot;
import frc.robot.autonomous.AutonomousConstants;

public class ShootOnMoveSequence extends ParallelCommandGroup {

	public ShootOnMoveSequence(Robot robot, String pathName) {
		super(AutonomousConstants.PATHFIND_OR_FOLLOW_PATH_FUNCTION.apply(robot, pathName), AutonomousConstants.SHOOTING_COMMAND);
	}

}
