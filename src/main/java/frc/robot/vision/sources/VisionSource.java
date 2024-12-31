package frc.robot.vision.sources;

import frc.robot.vision.data.VisionData;
import frc.utils.Filter;

import java.util.Optional;

public interface VisionSource<ReturnType extends VisionData> {

	void update();

	Optional<ReturnType> getVisionData();

	Optional<ReturnType> getFilteredVisionData();

	Filter<ReturnType> setFilter(Filter<ReturnType> newFilter);

}

