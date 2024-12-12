package frc.robot.vision.sources;

import frc.robot.vision.filters.Filter;
import frc.robot.vision.rawdata.IRawVisionData;
import frc.robot.vision.rawdata.RawVisionData;

import java.util.Optional;

public interface VisionSource {

	void update();

	Optional<IRawVisionData> getRawVisionData();

	boolean shouldDataBeFiltered();

	Filter setFilter(Filter newFilter);

}
