package frc.utils.auto;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import java.util.List;

public class AutonomousChooser {

	private final LoggedDashboardChooser<GBAuto> chooser;

	public AutonomousChooser(String name, List<GBAuto> autoList) {
		this.chooser = new LoggedDashboardChooser<>(name);
		autoList.forEach(auto -> chooser.addOption(auto.getName(), auto));
		chooser.addDefaultOption("None", new GBAuto());
	}

	public GBAuto getChosenValue() {
		return chooser.get();
	}

}
