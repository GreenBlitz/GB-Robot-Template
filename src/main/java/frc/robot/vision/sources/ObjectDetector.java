package frc.robot.vision.sources;

import frc.robot.vision.data.ObjectData;
import frc.utils.Filter;

import java.util.ArrayList;
import java.util.Optional;

public interface ObjectDetector {

	void update();

	ArrayList<ObjectData> getAllObjectData();

	private Optional<ObjectData> filterObjectData(ObjectData objectData) {
		if (getFilter().apply(objectData)) {
			return Optional.of(objectData);
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
		double minDistance = allFilteredObjectData.get(0).getEstimatedPose().toPose2d().getTranslation().getNorm();

		for (ObjectData objectData : allFilteredObjectData) {
			double distance = objectData.getEstimatedPose().toPose2d().getTranslation().getNorm();
			if (distance < minDistance) {
				minDistance = distance;
				closestObject = objectData;
			}
		}
		return closestObject;
	}

	Optional<ObjectData> getClosestObjectData();

	default Optional<ObjectData> getFilteredClosestObjectData() {
		return filterObjectData(getClosestObjectData());
	}

	void setFilter(Filter<? super ObjectData> newFilter);

	Filter<? super ObjectData> getFilter();

	default void clearFilter() {
		setFilter(Filter.nonFilteringFilter());
	}

}
