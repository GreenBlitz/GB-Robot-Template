package frc.robot.poseestimator.linearfilters;

import edu.wpi.first.math.filter.LinearFilter;

public abstract class GBLinearFilter {

	double calculateNewData(double newData) {
		return getFilter().calculate(newData);
	};

	double getLatestCalculation(double input) {
		return getFilter().lastValue();
	};

	abstract LinearFilter getFilter();

}
