package frc.utils.brakestate;

import frc.utils.GBSubsystem;
import frc.utils.MotorSubsystem;

import java.util.ArrayList;

public class BrakeStateManager {

	public static final ArrayList<MotorSubsystem> subsystems = new ArrayList<>();

	public static void addSubsystem(MotorSubsystem subsystem) {
		subsystems.add(subsystem);
	}

	public static void brake() {
		for (MotorSubsystem subsystem : subsystems) {
			subsystem.setBrake(true);
		}
	}

	public static void coast() {
		for (MotorSubsystem subsystem : subsystems) {
			subsystem.setBrake(false);
		}
	}

}
