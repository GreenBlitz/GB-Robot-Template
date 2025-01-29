package frc.utils.linearfilters;

import org.littletonrobotics.junction.Logger;

import java.util.ArrayList;

/**
 * A class for automatic updating of linear filters. Its periodic shall run ASAP. Has the functionality to be disabled in case of taking too much
 * resources.
 */
public class LinearFiltersManager {

	private static boolean enabled = true;

	private static final ArrayList<IPeriodicLinearFilter> periodicLinearFilters = new ArrayList<>();

	public static void periodic(String logPath) {
		Logger.recordOutput(logPath + "/enabled", enabled);
		for (IPeriodicLinearFilter filter : periodicLinearFilters) {
			filter.update();
			filter.log(logPath);
		}
	}

	public static void addFilter(IPeriodicLinearFilter filter) {
		periodicLinearFilters.add(filter);
	}

	public static void resetAllFilters() {
		periodicLinearFilters.forEach(IPeriodicLinearFilter::hardReset);
	}

	public static void disable() {
		enabled = false;
	}

	public static void enabled() {
		enabled = true;
	}

	public static boolean isEnabled() {
		return enabled;
	}

}
