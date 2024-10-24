package frc.robot.poseestimator.helpers;


import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;

public class ObservationCountHelper<T> {

	private final Supplier<List<T>> observationSupplier;
	private final int maxCount;
	private final List<T> stackedObservations;
	private int observationsCount;

	public ObservationCountHelper(Supplier<List<T>> observationSupplier, int maxCount) {
		this.observationSupplier = observationSupplier;
		this.maxCount = maxCount;
		stackedObservations = new ArrayList<>();
		observationsCount = 0;
	}

	public List<T> getStackedObservations() {
		if (observationsCount == maxCount) {
			observationsCount = 0;
			return stackedObservations;
		} else {
			stackedObservations.addAll(observationSupplier.get());
			observationsCount++;
		}
		return new ArrayList<>();
	}

}
