package frc.robot.vision.sources;

import frc.robot.vision.data.VisionData;
import frc.utils.Filter;

import java.util.Optional;
import java.util.function.BiFunction;

public interface VisionSource<ReturnType extends VisionData> {

	void update();

	Optional<ReturnType> getVisionData();

	Optional<ReturnType> getFilteredVisionData();

	Filter<ReturnType> setFilter(Filter<ReturnType> newFilter);

	default Filter<ReturnType> clearFilter() {
		return setFilter(new Filter<>(data -> true));
	}

	Filter<ReturnType> applyOnFilter(
		BiFunction<Filter<ReturnType>, Filter<ReturnType>, Filter<ReturnType>> applicationFunction,
		Filter<ReturnType> filterToApplyWith
	);

}
