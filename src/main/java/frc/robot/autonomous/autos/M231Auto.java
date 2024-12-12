package frc.robot.autonomous.autos;

import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Robot;
import frc.robot.autonomous.AutonomousConstants;
import frc.robot.autonomous.sequences.IntakeShootSequence;
import frc.utils.auto.PathPlannerUtils;

public class M231Auto extends PathPlannerAuto {

	public M231Auto(Robot robot) {
		super(
			new SequentialCommandGroup(
				AutonomousConstants.SHOOTING_COMMAND.get(),
				new IntakeShootSequence(robot, "M2"),
				new IntakeShootSequence(robot, "23"),
				new IntakeShootSequence(robot, "31")
			),
			PathPlannerUtils.getPathStartingPose("M2")
		);
	}

}
