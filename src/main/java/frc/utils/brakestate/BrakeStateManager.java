package frc.utils.brakestate;


import java.util.ArrayList;
import java.util.List;

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

    private static void setBrakeMode(BrakeMode wantedMode, List<Runnable> setModes) {
        if (mode == wantedMode) {
            return;
        }
        mode = wantedMode;
        for (Runnable setMode : setModes) {
            setMode.run();
        }
    }

	public static void brake() {
        setBrakeMode(BrakeMode.BRAKE, brakeRunnables);
	}

	public static void coast() {
		setBrakeMode(BrakeMode.COAST, coastRunnables);
	}

}
