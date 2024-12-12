package frc.robot.vision.sources;

import frc.robot.vision.filters.Filter;
import frc.robot.vision.rawdata.RawVisionData;

import java.util.Optional;

public interface VisionSource<ReturnedData extends RawVisionData> {

	void update();

	Optional<ReturnedData> getRawVisionData();

	boolean shouldDataBeFiltered();

	Filter<ReturnedData> setFilter(Filter<ReturnedData> newFilter);

}
