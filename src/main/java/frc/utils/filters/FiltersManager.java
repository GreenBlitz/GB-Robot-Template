package frc.utils.filters;

import frc.utils.filters.linearfilters.IPeriodicFilter;
import org.littletonrobotics.junction.Logger;

import java.util.ArrayList;

/**
 * A class for automatic updating of filters. Its periodic shall run ASAP. Has the functionality to be disabled in case of taking too much
 * resources.
 */
public class FiltersManager {

	private static boolean enabled = true;

	private static final ArrayList<IPeriodicFilter<?>> periodicLinearFilters = new ArrayList<>();

	public static void periodic(String logPath) {
		Logger.recordOutput(logPath + "/enabled", enabled);
		for (IPeriodicFilter<?> filter : periodicLinearFilters) {
			filter.update();
			filter.log(logPath);
		}
	}

	public static void addFilter(IPeriodicFilter<?> filter) {
		periodicLinearFilters.add(filter);
	}

	public static void resetAllFilters() {
		periodicLinearFilters.forEach(IPeriodicFilter::hardReset);
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
