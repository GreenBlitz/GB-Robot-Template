package frc.robot.poseestimator.helpers;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;
public class ObservationAccumulator<T> {

	private final Supplier<List<T>> observationsSupplier;
	private final int maxDataIntakes;
	private List<T> accumulatedObservations;
	private int dataIntakeCounter;

	public ObservationAccumulator(Supplier<List<T>> observationSupplier, int maxCount) {
		this .observationsSupplier = observationSupplier;
		this.maxDataIntakes = maxCount;
		this .accumulatedObservations = new ArrayList<>();
		this .dataIntakeCounter = 0;
	}

	public List<T> getAccumulatedList() {
		accumulatedObservations.addAll(observationsSupplier.get());
		dataIntakeCounter++;

		if (dataIntakeCounter >= maxDataIntakes) {
			dataIntakeCounter = 0;
			List<T> stackedObservationsCopy  = accumulatedObservations;
			accumulatedObservations = new ArrayList<>();
			return stackedObservationsCopy ;
		}
		return new ArrayList<>();

	}

	public int getCount() {
		return dataIntakeCounter;
	}

}
