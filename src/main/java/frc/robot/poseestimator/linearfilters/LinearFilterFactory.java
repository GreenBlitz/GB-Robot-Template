package frc.robot.poseestimator.linearfilters;

public class LinearFilterFactory {

	public static ILinearFilter create(LinearFilterType filterType, String parentLogPath, double modifier) {
		return switch (filterType) {
			case highPassIIR -> new LinearFilters.HighPassIIR(modifier);
			case singlePoleIIR -> new LinearFilters.SinglePoleIIR(modifier);
			case movingAverageFIR -> new LinearFilters.MovingAverageFIR((int) modifier);
		};
	}

}
