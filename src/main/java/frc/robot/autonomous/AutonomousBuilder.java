package frc.robot.autonomous;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Robot;
import frc.utils.auto.PathPlannerUtils;

import java.util.HashMap;
import java.util.Map;

public class AutonomousBuilder {

	public static Map<String, Command> getAllAutos(Robot robot) {
		Map<String, Command> autoMap = new HashMap<>();
		autoMap.put("M231", withResetOdometry(M231(robot)));
		autoMap.put("Rotate", new PathPlannerAuto("Rotate"));
		autoMap.put("Rotate 2m", new PathPlannerAuto("Rotate 2m"));
		autoMap.put("Straight 2m", new PathPlannerAuto("Straight 2m"));
		return autoMap;
	}

	private static Command withResetOdometry(PathPlannerAuto auto) {
		return AutoBuilder.resetOdom(auto.getStartingPose()).andThen(auto);
	}

	private static PathPlannerAuto M231(Robot robot) {
		String logPath = AutonomousConstants.LOG_PATH_PREFIX + "M231/";
		return new PathPlannerAuto(
			new SequentialCommandGroup(
				AutonomousConstants.SHOOTING_COMMAND.get(),
				SequencesBuilder.IntakeShoot(robot, "M2", logPath),
				SequencesBuilder.IntakeShoot(robot, "23", logPath),
				SequencesBuilder.IntakeShoot(robot, "31", logPath)
			),
			PathPlannerUtils.getPathStartingPose(PathPlannerUtils.getPathFromFile("M2", logPath))
		);
	}

}
