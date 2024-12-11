package frc.robot.autonomous.sequences;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import frc.robot.Robot;
import frc.robot.autonomous.AutonomousConstants;

public class IntakeSequence extends ParallelRaceGroup {

	public IntakeSequence(Robot robot, String pathName) {
		super(AutonomousConstants.PATHFIND_OR_FOLLOW_PATH_FUNCTION.apply(robot, pathName), AutonomousConstants.INTAKE_COMMAND);
	}

}
