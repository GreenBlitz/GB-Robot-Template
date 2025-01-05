package frc.robot.vision.sources;

import frc.utils.Filter;

public class VisionSourceConstants {

	public static <T> Filter<T> NON_FILTERING_FILTER() {
		return new Filter<>(data -> true);
	}

}
