package frc.utils.brakestate;


import java.util.ArrayList;

public class BrakeStateManager {

	private enum BrakeMode {
		UNKNOWN,
		BRAKE,
		COAST
	}

	private static final ArrayList<Runnable> brakeRunnables = new ArrayList<>();
	private static final ArrayList<Runnable> coastRunnables = new ArrayList<>();
	private static BrakeMode mode = BrakeMode.UNKNOWN;

	public static void add(Runnable brake, Runnable coast) {
		brakeRunnables.add(brake);
		coastRunnables.add(coast);
	}

	public static void brake() {
		if (mode == BrakeMode.BRAKE) {
			return;
		}
		mode = BrakeMode.BRAKE;
		for (Runnable brake : brakeRunnables) {
			brake.run();
		}
	}

	public static void coast() {
		if (mode == BrakeMode.COAST) {
			return;
		}
		mode = BrakeMode.COAST;
		for (Runnable coast : coastRunnables) {
			coast.run();
		}
	}

}
