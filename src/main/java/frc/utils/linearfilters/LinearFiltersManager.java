package frc.utils.linearfilters;

import java.util.ArrayList;
import java.util.List;

/**
 * A class for automatic updating of linear filters. Its periodic run first.
 */
public class LinearFiltersManager {

	private static final ArrayList<PeriodicLinearFilter> periodicLinearFilters = new ArrayList<>();

	public static void periodic(String logPath) {
		for (PeriodicLinearFilter filter : periodicLinearFilters) {
			filter.update();
			filter.log(logPath);
		}
	}

	public static void addFilter(PeriodicLinearFilter filter) {
		periodicLinearFilters.add(filter);
	}

	public static void resetAllFilters() {
		periodicLinearFilters.forEach(PeriodicLinearFilter::hardReset);
	}

}
