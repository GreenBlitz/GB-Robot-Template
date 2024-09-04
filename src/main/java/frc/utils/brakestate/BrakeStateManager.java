package frc.utils.brakestate;

import frc.utils.MotorSubsystem;

import java.util.ArrayList;

public class BrakeStateManager {

	public static final ArrayList<Runnable> coastRunnables = new ArrayList<>();
	public static final ArrayList<Runnable> brakeRunnables = new ArrayList<>();

	public static void add(Runnable brake, Runnable coast) {
		coastRunnables.add(coast);
		brakeRunnables.add(brake);
	}

	public static void brake() {
		for (Runnable brakeRunnable : brakeRunnables) {
			brakeRunnable.run();
		}
	}

	public static void coast() {
		for (Runnable coastRunnable : coastRunnables) {
			coastRunnable.run();
		}
	}

}
