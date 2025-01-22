package frc.robot.vision;

import edu.wpi.first.math.Vector;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.numbers.N3;
import frc.robot.vision.data.AprilTagVisionData;
import frc.utils.linearfilters.IPeriodicLinearFilter;
import frc.utils.linearfilters.LinearFiltersManager;
import frc.utils.linearfilters.Periodic3DlLinearFilter;
import frc.utils.pose.PoseUtils;
import org.littletonrobotics.junction.Logger;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;

public class JumpingDetector implements IPeriodicLinearFilter {

	private final Supplier<ArrayList<AprilTagVisionData>> visionSupplier;
	private final Periodic3DlLinearFilter filter;
	private final String name;
	public final int taps;
	private final ArrayList<Vector<N3>> innerData;

	private double dataStdDevs;
	private int emptyCount = 0;
	private List<Vector<N3>> visionOutputs;

	public static double STABLE_TOLERANCE = 0.07;
	public static int MAX_COUNT_VALUE = 10;
	public static double MAX_JUMP_SIZE = 0.05;

	public JumpingDetector(
		ArrayList<Periodic3DlLinearFilter> filters,
		Supplier<ArrayList<AprilTagVisionData>> visionSupplier,
		int taps,
		String name
	) {
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
		this.dataStdDevs = 0;

		LinearFiltersManager.addFilter(filter);
	}

	@Override
	public void hardReset() {
		filter.hardReset();
	}

	@Override
	public void log(String parentLogPath) {
		String logPath = parentLogPath + name;
		Logger.recordOutput(logPath + "stdDevs", dataStdDevs);
		Logger.recordOutput(logPath + "distFromLast", getFilterResult().minus(innerData.get(innerData.size() - 1)).norm());
		Logger.recordOutput(logPath + "filterResult", getFilterResult());
		Logger.recordOutput(logPath + "idDeterminable", isDeterminable());
	}

	@Override
	public void update() {
		List<Vector<N3>> asPose2D = visionSupplier.get()
			.stream()
			.map(x -> PoseUtils.poseToVector.apply(x.getEstimatedPose().toPose2d()))
			.toList();
		innerData.addAll(asPose2D);
		visionOutputs = asPose2D;
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
		dataStdDevs = PoseUtils.stdDevVector(innerData).norm();
	}

	private boolean isDeterminable() {
		return dataStdDevs < STABLE_TOLERANCE && emptyCount > MAX_COUNT_VALUE && !innerData.isEmpty();
	}

	public List<AprilTagVisionData> getFilteredData() {
		return visionSupplier.get()
			.stream()
			.filter(data -> getFilterResult().minus(PoseUtils.poseToVector.apply(data.getEstimatedPose().toPose2d())).norm() > MAX_JUMP_SIZE)
			.toList();
	}

	public Vector<N3> getFilterResult() {
		return filter.getAsColumnVector();
	}

}
