package frc.utils.auto;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import java.util.List;
import java.util.function.Supplier;

public class AutonomousChooser {

	private final LoggedDashboardChooser<Supplier<GBAuto>> chooser;

	public AutonomousChooser(String name, List<Supplier<GBAuto>> autoList) {
		this.chooser = new LoggedDashboardChooser<>(name);
		autoList.forEach(auto -> chooser.addOption(auto.get().getName(), auto));
		chooser.addDefaultOption("None", GBAuto::new);
	}

	public GBAuto getChosenValue() {
		return chooser.get().get();
	}

	public boolean isDefaultOption() {
		return chooser.getSendableChooser().getSelected().equals("None");
	}

}
