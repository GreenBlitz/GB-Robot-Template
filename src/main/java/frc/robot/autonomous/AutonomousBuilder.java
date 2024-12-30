package frc.robot.autonomous;

import com.pathplanner.lib.commands.PathPlannerAuto;
import frc.robot.Robot;

import java.util.List;

public class AutonomousBuilder {

	public static List<PathPlannerAuto> getAllAutos(Robot robot) {
		return List.of(new PathPlannerAuto("Rotate"), new PathPlannerAuto("Rotate 2m"), new PathPlannerAuto("Straight 2m"));
	}

}
