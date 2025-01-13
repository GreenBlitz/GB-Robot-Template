package frc.robot.vision.sources;

import frc.robot.vision.data.VisionData;
import frc.utils.Filter;

import java.util.Optional;
import java.util.function.Function;

public interface VisionSource<T extends VisionData> {

	void update();

	Optional<T> getVisionData();

	default Optional<T> getFilteredVisionData() {
		Optional<T> visionData = getVisionData();
		if (getVisionData().isPresent() && getFilter().apply(visionData.get())) {
			return visionData;
		}
		return Optional.empty();
	}

	void setFilter(Filter<T> newFilter);

	Filter<T> getFilter();

	default void clearFilter() {
		setFilter(Filter.nonFilteringFilter());
	}

	default void applyFunctionOnFilter(Function<Filter<T>, Filter<T>> filterChangingFunction) {
		setFilter(filterChangingFunction.apply(getFilter()));
	}

}
