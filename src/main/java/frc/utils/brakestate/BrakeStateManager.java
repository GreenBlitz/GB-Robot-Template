package frc.utils.brakestate;


import java.util.ArrayList;

public class BrakeStateManager {

	private static final ArrayList<Runnable> brakeRunnables = new ArrayList<>();
	private static final ArrayList<Runnable> coastRunnables = new ArrayList<>();
	private static boolean isBrake = false;

	public static void add(Runnable brake, Runnable coast) {
		brakeRunnables.add(brake);
		coastRunnables.add(coast);
	}

	public static void brake() {
		if (isBrake) {
			return;
		}
		isBrake = true;
		for (Runnable brake : brakeRunnables) {
			brake.run();
		}
	}

	public static void coast() {
		if (!isBrake) {
			return;
		}
		isBrake = false;
		for (Runnable coast : coastRunnables) {
			coast.run();
		}
	}

}
