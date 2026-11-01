package frc.utils.auto;

import org.littletonrobotics.junction.networktables.LoggedNetworkChooser;

import java.util.List;
import java.util.function.Supplier;

public class AutonomousChooser {

	private final Supplier<PathPlannerAutoWrapper> defaultOption = PathPlannerAutoWrapper::new;
	private final LoggedNetworkChooser<Supplier<PathPlannerAutoWrapper>> chooser;

	public AutonomousChooser(String name, List<Supplier<PathPlannerAutoWrapper>> autoList) {
		this.chooser = new LoggedNetworkChooser<>(name);
		autoList.forEach(auto -> chooser.add(auto.get().getName(), auto));
		chooser.addDefault("None", defaultOption);
	}

	public PathPlannerAutoWrapper getChosenValue() {
		return chooser.get().get();
	}

	public LoggedNetworkChooser<Supplier<PathPlannerAutoWrapper>> getChooser() {
		return chooser;
	}

	public boolean isDefaultOptionChosen() {
		return chooser.get() == defaultOption;
	}

}
