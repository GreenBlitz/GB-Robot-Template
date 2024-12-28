package frc.robot.poseestimator.helpers;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;

public class DataAccumulator<T> {

	private final Supplier<List<T>> dataSupplier;
	private final int maxDataIntakes;
	private List<T> accumulatedData;
	private int dataIntakeCounter;

	public DataAccumulator(Supplier<List<T>> dataSupplier, int maxDataIntakes) {
		this.dataSupplier = dataSupplier;
		this.maxDataIntakes = maxDataIntakes;
		this.accumulatedData = new ArrayList<>();
		this.dataIntakeCounter = 0;
	}

	public List<T> getAccumulatedList() {
		accumulatedData.addAll(dataSupplier.get());
		dataIntakeCounter++;

		if (dataIntakeCounter >= maxDataIntakes) {
			dataIntakeCounter = 0;
			List<T> accumulatedDataCopy = accumulatedData;
			accumulatedData = new ArrayList<>();
			return accumulatedDataCopy;
		}
		return new ArrayList<>();
	}

	public int getIntakeCount() {
		return dataIntakeCounter;
	}

}
