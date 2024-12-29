package frc.utils.auto;

import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.wpilibj2.command.Commands;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import java.util.List;

public class AutonomousChooser {

	private final LoggedDashboardChooser<PathPlannerAuto> chooser;

	public AutonomousChooser(String name, List<PathPlannerAuto> autoList) {
		this.chooser = new LoggedDashboardChooser<>(name);
		autoList.forEach(auto -> chooser.addOption(auto.getName(), auto));
		chooser.addDefaultOption("None", new PathPlannerAuto(Commands.none()));
	}

	public PathPlannerAuto getChosenValue() {
		return chooser.get();
	}

}
