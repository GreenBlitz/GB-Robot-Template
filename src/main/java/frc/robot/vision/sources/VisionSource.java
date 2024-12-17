package frc.robot.vision.sources;

import frc.robot.vision.filters.Filter;
import frc.robot.vision.rawdata.VisionData;

import java.util.Optional;

public interface VisionSource<ReturnType extends VisionData> {

	void update();

	Optional<ReturnType> getVisionData();

	Optional<ReturnType> getFilteredVisionData();

	Filter<ReturnType> setFilter(Filter<ReturnType> newFilter);

}
