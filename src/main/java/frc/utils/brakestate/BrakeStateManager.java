package frc.utils.brakestate;


import org.littletonrobotics.junction.Logger;

import java.util.ArrayList;

public class BrakeStateManager {

	private static final ArrayList<Runnable> brakeRunnables = new ArrayList<>();
	private static final ArrayList<Runnable> coastRunnables = new ArrayList<>();

	public static void add(Runnable brake, Runnable coast) {
		brakeRunnables.add(brake);
		coastRunnables.add(coast);
	}

	public static void brake() {
		Logger.recordOutput("start", true);
		for (Runnable brake : brakeRunnables) {
			brake.run();
		}
	}

	public static void coast() {
		for (Runnable coast : coastRunnables) {
			coast.run();
		}
	}

}
