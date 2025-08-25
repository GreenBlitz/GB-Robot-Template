package frc.utils.brakestate;


import java.util.ArrayList;

public class BrakeStateManager {

	private static final ArrayList<Runnable> brakeRunnables = new ArrayList<>();
	private static final ArrayList<Runnable> coastRunnables = new ArrayList<>();
	private static boolean isBrake = false;
	private static boolean isFirst = true;

	public static void add(Runnable brake, Runnable coast) {
		brakeRunnables.add(brake);
		coastRunnables.add(coast);
	}

	public static void brake() {
		if (!isFirst && isBrake) {
			return;
		}
		isFirst = false;
		isBrake = true;
		for (Runnable brake : brakeRunnables) {
			brake.run();
		}
	}

	public static void coast() {
		if (!isFirst && !isBrake) {
			return;
		}
		isFirst = false;
		isBrake = false;
		for (Runnable coast : coastRunnables) {
			coast.run();
		}
	}

}
