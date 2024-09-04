package frc.utils.brakestate;


import frc.utils.MotorSubsystem;

import java.util.ArrayList;

public class BrakeStateManager {

	private static final ArrayList<MotorSubsystem> motorSubsystems = new ArrayList<>();

	public static void add(MotorSubsystem motorSubsystem) {
		motorSubsystems.add(motorSubsystem);
	}

	public static void brake() {
		for (MotorSubsystem motorSubsystem : motorSubsystems) {
			motorSubsystem.setBrake(true);
		}
	}

	public static void coast() {
		for (MotorSubsystem motorSubsystem : motorSubsystems) {
			motorSubsystem.setBrake(false);
		}
	}

}
