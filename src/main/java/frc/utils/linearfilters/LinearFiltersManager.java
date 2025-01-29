package frc.utils.linearfilters;

import org.littletonrobotics.junction.Logger;

import java.util.ArrayList;

/**
 * A class for automatic updating of linear filters. Its periodic shall run ASAP.
 */
public class LinearFiltersManager {

	private static final ArrayList<IPeriodicLinearFilter> periodicLinearFilters = new ArrayList<>();

	public static void periodic(String logPath) {
		Logger.recordOutput(logPath + "/running", true);
		Logger.recordOutput(logPath + "/size", periodicLinearFilters.size());
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

}
