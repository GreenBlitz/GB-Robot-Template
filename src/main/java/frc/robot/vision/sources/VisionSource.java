package frc.robot.vision.sources;

import frc.robot.vision.data.AprilTagVisionData;
import frc.robot.vision.data.VisionData;
import frc.utils.Filter;

import java.util.Optional;
import java.util.function.BiFunction;
import java.util.function.Function;

public interface VisionSource<ReturnType extends VisionData> {

	void update();

	Optional<ReturnType> getVisionData();

	Optional<ReturnType> getFilteredVisionData();

	void setFilter(Filter<ReturnType> newFilter);

	Filter<ReturnType> getFilter();

	default void clearFilter() {
		setFilter(Filter.nonFilteringFilter());
	}

	default void applyFunctionOnFilter(Function<Filter<ReturnType>, Filter<ReturnType>> filterChangingFunction) {
		setFilter(filterChangingFunction.apply(getFilter()));
	}

}
