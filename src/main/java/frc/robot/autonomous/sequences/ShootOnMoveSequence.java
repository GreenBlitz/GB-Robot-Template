package frc.robot.autonomous.sequences;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Robot;
import frc.robot.autonomous.AutonomousConstants;
import frc.utils.auto.PathPlannerUtils;

public class ShootOnMoveSequence extends ParallelCommandGroup {

	public ShootOnMoveSequence(Robot robot, String pathName) {
		super(PathPlannerUtils.followPathOrDriveToPathEnd(robot, pathName), AutonomousConstants.SHOOTING_COMMAND.get());
	}

}
