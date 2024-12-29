package frc.utils.auto;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import java.util.List;

public class AutonomousChooser {

	private final LoggedDashboardChooser<Command> chooser;

	public AutonomousChooser(String name, List<Command> autoList) {
		this.chooser = new LoggedDashboardChooser<>(name);
		autoList.forEach(auto -> chooser.addOption(auto.getName(), auto));
		chooser.addDefaultOption("None", Commands.none());
	}

	public Command getChosenValue() {
		return chooser.get();
	}

}
