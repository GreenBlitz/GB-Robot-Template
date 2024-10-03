package frc.robot.poseestimator.linearfilters;

import java.util.NavigableSet;

public interface ILinearFilter {

	double applyFilterOnData(NavigableSet<Double> data);

}
