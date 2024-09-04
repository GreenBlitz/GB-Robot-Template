package frc.utils.brakestate;


import java.util.ArrayList;

public class BrakeStateManager {

	private static final ArrayList<Runnable> coastRunnables = new ArrayList<>();
	private static final ArrayList<Runnable> brakeRunnables = new ArrayList<>();

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
