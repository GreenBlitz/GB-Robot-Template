package frc.robot.vision.sources;

import frc.robot.vision.data.ObjectData;
import frc.utils.Filter;

import java.util.ArrayList;
import java.util.Optional;

public interface ObjectDetector {

	void update();

	ArrayList<Optional<ObjectData>> getAllObjectData();

	private Optional<ObjectData> getFilteredObjectData(Optional<ObjectData> objectData) {
		if (objectData.isPresent() && getFilter().apply(objectData.get())) {
			return objectData;
		}
		return Optional.empty();
	}

	default ArrayList<Optional<ObjectData>> getAllFilteredObjectData() {
		ArrayList<Optional<ObjectData>> allObjectData = getAllObjectData();
		allObjectData.replaceAll(this::getFilteredObjectData);
		return allObjectData;
	}

	Optional<ObjectData> getClosestFilteredObjectData();

	void setFilter(Filter<? super ObjectData> newFilter);

	Filter<? super ObjectData> getFilter();

	default void clearFilter() {
		setFilter(Filter.nonFilteringFilter());
	}

}
