package frc.robot.vision.sources;

import frc.robot.vision.data.VisionData;
import frc.utils.Filter;

import java.util.Optional;
import java.util.function.BiFunction;

public interface VisionSource<ReturnType extends VisionData> {

	void update();

	Optional<ReturnType> getVisionData();

	Optional<ReturnType> getFilteredVisionData();

	void setFilter(Filter<ReturnType> newFilter);

	default void clearFilter() {
		setFilter(VisionSourceConstants.getNonFilteringFilter());
	}

	void applyOnFilter(
		BiFunction<Filter<ReturnType>, Filter<ReturnType>, Filter<ReturnType>> applicationFunction,
		Filter<ReturnType> filterToApplyWith
	);

}
