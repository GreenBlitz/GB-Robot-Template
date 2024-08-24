package frc.utils.breakState;

import frc.utils.GBSubsystem;

import java.util.ArrayList;

public class BrakeStateManager {

	private static ArrayList<GBSubsystem> subsystems = new ArrayList<>();

	public static void addSubsystem(GBSubsystem subsystem) {
		subsystems.add(subsystem);
	}

	public static void brake() {
		for (GBSubsystem subsystem : subsystems) {
			subsystem.setBrake(true);
		}
	}

	public static void coast() {
		for (GBSubsystem subsystem : subsystems) {
			subsystem.setBrake(false);
		}
	}

}
