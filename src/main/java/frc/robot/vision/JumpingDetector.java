package frc.robot.vision;

import edu.wpi.first.math.Vector;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.numbers.N3;
import frc.utils.linearfilters.IPeriodicLinearFilter;
import frc.utils.linearfilters.LinearFiltersManager;
import frc.utils.linearfilters.Periodic3DlLinearFilter;
import frc.utils.pose.PoseUtils;

import java.util.ArrayList;
import java.util.function.Supplier;

public class JumpingDetector implements IPeriodicLinearFilter {

	private final Supplier<ArrayList<Vector<N3>>> visionSupplier;
	private final Periodic3DlLinearFilter filter;
	private final String name;
	public final int taps;
	private final ArrayList<Vector<N3>> innerData;

	private int emptyCount = 0;
	private ArrayList<Vector<N3>> visionOutputs;

	public static double STABLE_TOLERANCE = 0.07;
	public static int MAX_COUNT_VALUE = 10;
	public static double MAX_JUMP_SIZE = 0.05;

	public JumpingDetector(ArrayList<Periodic3DlLinearFilter> filters, Supplier<ArrayList<Vector<N3>>> visionSupplier, int taps, String name) {
		this.visionSupplier = visionSupplier;
		this.name = name;
		this.visionOutputs = new ArrayList<>();
		this.filter = new Periodic3DlLinearFilter(
			() -> visionOutputs,
			LinearFilter.movingAverage(taps),
			LinearFilter.movingAverage(taps),
			LinearFilter.movingAverage(taps),
			name + "Inner"
		);
		this.innerData = new ArrayList<>();
		this.taps = taps;

		LinearFiltersManager.addFilter(filter);
	}

	@Override
	public void hardReset() {
		filter.hardReset();
	}

	@Override
	public void log(String parentLogPath) {}

	@Override
	public void update() {
		innerData.addAll(visionSupplier.get());
		visionOutputs = visionSupplier.get();
		if (visionSupplier.get().isEmpty()) {
			emptyCount++;
		} else {
			emptyCount = 0;
			innerData.clear();
		}
		if (emptyCount > MAX_COUNT_VALUE) {
			hardReset();
		}
		while (innerData.size() >= taps) {
			innerData.remove(innerData.size() - 1);
		}
	}

	private boolean isFilterStable() {
		return !determinable() || getFilterResult().minus(innerData.get(innerData.size() - 1)).norm() > MAX_JUMP_SIZE;
	}

	private boolean determinable() {
		return PoseUtils.stdDevVector(innerData).norm() < STABLE_TOLERANCE;
	}

	public boolean filterData() {
		return isFilterStable() && emptyCount > MAX_COUNT_VALUE && !innerData.isEmpty();
	}

	public Vector<N3> getFilterResult() {
		return filter.getAsColumnVector();
	}

}
