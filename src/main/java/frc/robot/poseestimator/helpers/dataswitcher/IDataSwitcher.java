package frc.robot.poseestimator.helpers.dataswitcher;


import java.util.Optional;

/**
 * The <code>DataSwitcher</code> class is being used to switch between two data sources, over time, without creating a "bump" in the output.
 */
public interface IDataSwitcher<T> {

	Optional<T> getValue(double time);

	void switchSources();

	void switchToFirstSource();

	void switchToSecondsSource();

	boolean isFirstSourceBeingUsed();

}
