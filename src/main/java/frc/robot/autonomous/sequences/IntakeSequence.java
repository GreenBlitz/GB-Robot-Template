package frc.robot.autonomous.sequences;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import frc.robot.Robot;
import frc.robot.autonomous.AutonomousConstants;
import frc.utils.auto.PathPlannerUtils;

public class IntakeSequence extends ParallelRaceGroup {

	public IntakeSequence(Robot robot, String pathName) {
		super(PathPlannerUtils.followPathOrDriveToPathEnd(robot, pathName), AutonomousConstants.INTAKE_COMMAND.get());
	}

}
