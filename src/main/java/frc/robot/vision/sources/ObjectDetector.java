package frc.robot.vision.sources;

import frc.robot.vision.data.ObjectData;
import frc.utils.Filter;

import java.util.ArrayList;
import java.util.Optional;

public interface ObjectDetector {

	ArrayList<ObjectData> getAllObjectData();

	Optional<ObjectData> getClosestObjectData();

	void log();

	void update();

	void setFilter(Filter<? super ObjectData> newFilter);

	Filter<? super ObjectData> getFilter();

	private Optional<ObjectData> filterObjectData(ObjectData objectData) {
		if (getFilter().apply(objectData)) {
			return Optional.of(objectData);
		}
		return Optional.empty();
	}

	private Optional<ObjectData> filterObjectData(Optional<ObjectData> objectData) {
		if (objectData.isPresent() && getFilter().apply(objectData.get())) {
			return objectData;
		}
		return Optional.empty();
	}

	default ArrayList<ObjectData> getAllFilteredObjectData() {
		ArrayList<ObjectData> allObjectData = getAllObjectData();

		ArrayList<ObjectData> allFilteredObjectData = new ArrayList<>(0);
		for (ObjectData objectData : allObjectData) {
			filterObjectData(objectData).ifPresent(allFilteredObjectData::add);
		}
		return allFilteredObjectData;
	}

	default ObjectData getClosestFilteredObjectData() {
		ArrayList<ObjectData> allFilteredObjectData = getAllFilteredObjectData();
		ObjectData closestObject = allFilteredObjectData.get(0);
		double minDistance = allFilteredObjectData.get(0).getEstimatedPose().getNorm();

		for (ObjectData objectData : allFilteredObjectData) {
			double distance = objectData.getEstimatedPose().getNorm();
			if (distance < minDistance) {
				minDistance = distance;
				closestObject = objectData;
			}
		}
		return closestObject;
	}

	default Optional<ObjectData> getFilteredClosestObjectData() {
		return filterObjectData(getClosestObjectData());
	}

	default void clearFilter() {
		setFilter(Filter.nonFilteringFilter());
	}

}
