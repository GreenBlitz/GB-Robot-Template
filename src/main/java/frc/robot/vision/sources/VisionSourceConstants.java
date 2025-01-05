package frc.robot.vision.sources;

import frc.utils.Filter;

public class VisionSourceConstants {

	public static <T> Filter<T> getNonFilteringFilter() {
		return new Filter<>(data -> true);
	}

}
