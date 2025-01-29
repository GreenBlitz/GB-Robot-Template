package frc.utils.linearfilters;

import org.littletonrobotics.junction.Logger;

import java.util.ArrayList;

/**
 * A class for automatic updating of linear filters. Its periodic shall run ASAP.
 */
public class LinearFiltersManager {

	private static boolean running = true;

	private static final ArrayList<IPeriodicLinearFilter> periodicLinearFilters = new ArrayList<>();

	public static void periodic(String logPath) {
		Logger.recordOutput(logPath + "/running", running);
		if (running) {
			for (IPeriodicLinearFilter filter : periodicLinearFilters) {
				filter.update();
				filter.log(logPath);
			}
		}
	}

	public static void addFilter(IPeriodicLinearFilter filter) {
		periodicLinearFilters.add(filter);
	}

	public static void resetAllFilters() {
		periodicLinearFilters.forEach(IPeriodicLinearFilter::hardReset);
	}

	/**
	 * disables the periodic method. Can be used if takes too much resources.
	 */
	public static void disable() {
		running = false;
	}

	public static void enable() {
		running = true;
	}

}
