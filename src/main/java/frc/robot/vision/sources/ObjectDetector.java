package frc.robot.vision.sources;

import frc.robot.vision.data.ObjectData;
import frc.utils.Filter;

import java.util.ArrayList;
import java.util.Optional;

public interface ObjectDetector {

	void update();

	ArrayList<Optional<ObjectData>> getAllObjectData();

	private Optional<ObjectData> filterObjectData(Optional<ObjectData> objectData) {
		if (objectData.isPresent() && getFilter().apply(objectData.get())) {
			return objectData;
		}
		return Optional.empty();
	}

	default ArrayList<ObjectData> getAllFilteredObjectData() {
		ArrayList<Optional<ObjectData>> allOptionalFilteredObjectData = getAllObjectData();
		allOptionalFilteredObjectData.replaceAll(this::filterObjectData);

		ArrayList<ObjectData> allFilteredObjectData = new ArrayList<>(0);
		for (Optional<ObjectData> objectData : allOptionalFilteredObjectData) {
			objectData.ifPresent(allFilteredObjectData::add);
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
