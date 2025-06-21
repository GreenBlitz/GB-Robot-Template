package frc.robot.vision.objectdetection;

import frc.robot.vision.data.ObjectData;
import frc.utils.Filter;

import java.util.Optional;

public interface ObjectDetector {

	Optional<ObjectData> getClosestObjectData();

	void log();

	void update();

	void setFilter(Filter<? super ObjectData> newFilter);

	Filter<? super ObjectData> getFilter();

	private Optional<ObjectData> filterObjectData(Optional<ObjectData> objectData) {
		if (objectData.isPresent() && getFilter().apply(objectData.get())) {
			return objectData;
		}
		return Optional.empty();
	}

	default Optional<ObjectData> getFilteredClosestObjectData() {
		return filterObjectData(getClosestObjectData());
	}

	default void clearFilter() {
		setFilter(Filter.nonFilteringFilter());
	}

}
