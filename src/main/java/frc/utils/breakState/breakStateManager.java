package frc.utils.breakState;

import frc.utils.GBSubsystem;

import java.util.ArrayList;

public class breakStateManager {

	private static ArrayList<GBSubsystem> subsystemsList = new ArrayList<>();

	public static void addSubsystem(GBSubsystem subsystem) {
		subsystemsList.add(subsystem);
	}

	public static void breakState() {
		for (GBSubsystem subsystem : subsystemsList) {
			subsystem.setBreak();
		}
	}

	public static void coastState() {
		for (GBSubsystem subsystem : subsystemsList) {
			subsystem.setCoast();
		}
	}

}
