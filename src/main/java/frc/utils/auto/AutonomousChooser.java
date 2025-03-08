package frc.utils.auto;

import edu.wpi.first.wpilibj2.command.Command;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import java.util.List;
import java.util.function.Supplier;

public class AutonomousChooser {

	private final LoggedDashboardChooser<Supplier<Command>> chooser;

	public AutonomousChooser(String name, List<Supplier<Command>> autoList) {
		this.chooser = new LoggedDashboardChooser<>(name);
		autoList.forEach(auto -> chooser.addOption(auto.get().getName(), auto));
		chooser.addDefaultOption("None", PathPlannerAutoWrapper::new);
	}

	public Command getChosenValue() {
		return chooser.get().get();
	}

	public boolean isDefaultOptionChosen() {
		return chooser.getSendableChooser().getSelected().equals("None");
	}

}
