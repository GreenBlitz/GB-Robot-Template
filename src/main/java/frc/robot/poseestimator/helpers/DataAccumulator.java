package frc.robot.poseestimator.helpers;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;

public class DataAccumulator<T> {

	private final Supplier<List<T>> observationsSupplier;
	private final int maxDataIntakes;
	private List<T> accumulatedObservations;
	private int dataIntakeCounter;

	public DataAccumulator(Supplier<List<T>> observationSupplier, int maxCount) {
		this.observationsSupplier = observationSupplier;
		this.maxDataIntakes = maxCount;
		this.accumulatedObservations = new ArrayList<>();
		this.dataIntakeCounter = 0;
	}

	public List<T> getAccumulatedList() {
		accumulatedObservations.addAll(observationsSupplier.get());
		dataIntakeCounter++;

		if (dataIntakeCounter >= maxDataIntakes) {
			dataIntakeCounter = 0;
			List<T> accumulatedObservationsCopy = accumulatedObservations;
			accumulatedObservations = new ArrayList<>();
			return accumulatedObservationsCopy;
		}
		return new ArrayList<>();
	}

	public int getCount() {
		return dataIntakeCounter;
	}

}
