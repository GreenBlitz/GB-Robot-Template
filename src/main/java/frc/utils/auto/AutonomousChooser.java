package frc.utils.auto;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import java.util.Map;

public class AutonomousChooser {

	private final LoggedDashboardChooser<Command> chooser;

	public AutonomousChooser(String name, Map<String, Command> autoMap) {
		this.chooser = new LoggedDashboardChooser<>(name);
		autoMap.forEach(chooser::addOption);
		chooser.addDefaultOption("None", Commands.none());
	}

	public Command getChosenValue() {
		return chooser.get();
	}

}
