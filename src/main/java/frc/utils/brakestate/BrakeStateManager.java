package frc.utils.brakestate;


import org.littletonrobotics.junction.Logger;

import java.util.ArrayList;
import java.util.List;

public class BrakeStateManager {

	private static final ArrayList<Runnable> brakeRunnables = new ArrayList<>();
	private static final ArrayList<Runnable> coastRunnables = new ArrayList<>();
	private static BrakeMode currentMode = BrakeMode.UNKNOWN;

	public static void add(Runnable brake, Runnable coast) {
		brakeRunnables.add(brake);
		coastRunnables.add(coast);
	}

	private static void setBrakeMode(BrakeMode wantedMode, List<Runnable> setModes) {
		if (currentMode == wantedMode) {
			return;
		}
		currentMode = wantedMode;
		for (Runnable setMode : setModes) {
			setMode.run();
		}
	}

	public static void setBrakeMode(BrakeMode brakeMode) {
		switch (brakeMode) {
			case BRAKE -> setBrakeMode(BrakeMode.BRAKE, brakeRunnables);
			case COAST -> setBrakeMode(BrakeMode.COAST, coastRunnables);
		}
		log();
	}

	public static void log() {
		Logger.recordOutput("BrakeStateMangerState", currentMode);
	}

}
