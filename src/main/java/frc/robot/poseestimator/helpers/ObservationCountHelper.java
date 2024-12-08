package frc.robot.poseestimator.helpers;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;

public class ObservationCountHelper<T> {

	private final Supplier<List<T>> observationSupplier;
	private final int maxDataIntakeRepeats;
	private List<T> stackedObservations;
	private int dataIntakeCounter;

	public ObservationCountHelper(Supplier<List<T>> observationSupplier, int maxCount) {
		this.observationSupplier = observationSupplier;
		this.maxDataIntakeRepeats = maxCount;
		stackedObservations = new ArrayList<>();
		dataIntakeCounter = 0;
	}

	public List<T> getDataIntakelist() {
		dataIntakeCounter++;
		stackedObservations.addAll(observationSupplier.get());
		if (dataIntakeCounter == maxDataIntakeRepeats) {
			dataIntakeCounter = 0;
			List<T> copyOfStackedObservations = stackedObservations;
			stackedObservations = new ArrayList<>();
			return copyOfStackedObservations;
		} else {
			return new ArrayList<>();
		}
	}

	public int getCount() {
		return dataIntakeCounter;
	}

}
