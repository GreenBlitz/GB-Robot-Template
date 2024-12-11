package frc.robot.autonomous.sequences;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Robot;
import frc.robot.autonomous.AutonomousConstants;

public class IntakeShootSequence extends SequentialCommandGroup {

	public IntakeShootSequence(Robot robot, String pathName) {
		super(new IntakeSequence(robot, pathName), AutonomousConstants.SHOOTING_COMMAND);
	}

}
