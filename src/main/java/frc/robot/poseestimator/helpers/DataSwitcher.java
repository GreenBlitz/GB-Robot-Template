package frc.robot.poseestimator.helpers;

import frc.utils.time.TimeUtils;

import java.util.function.Function;
import java.util.function.Supplier;

/**
 * The <code>DataSwitcher</code> class is being used to switch between two data sources, over time, without creating a "bump" in the output.
 */
public class DataSwitcher {

	private final Supplier<Double> firstSource;
	private final Supplier<Double> secondSource;
	private final Function<Double, Double> timeToWeights;
	private final double timeToSwitch;
	private boolean useFirstSource;
	private double switchingTime;

	/**
	 * Constructor method for the <code>DataSwitcher</code>` class.
	 *
	 * @param firstSource:         A function time → ℝ. First and default source of continues data (till switchSource()).
	 * @param secondSource:        A function dom time → ℝ. First source of continues data.
	 * @param timeToWeights:       A surjective function time → [0, 1) that returns the weights in between the switches.
	 * @param timeToSwitchSeconds: The time required to switch.
	 */
	public DataSwitcher(
		Supplier<Double> firstSource,
		Supplier<Double> secondSource,
		Function<Double, Double> timeToWeights,
		double timeToSwitchSeconds
	) {
		this.firstSource = firstSource;
		this.secondSource = secondSource;
		this.timeToWeights = timeToWeights;
		this.timeToSwitch = timeToSwitchSeconds;
		this.useFirstSource = true;
		this.switchingTime = TimeUtils.getCurrentTimeSeconds();
	}

	public double getValue(double time) {
		double timeDelta = TimeUtils.getCurrentCycleTimeSeconds() - switchingTime;
		if (timeDelta == 1) {
			return useFirstSource ? firstSource.get() : secondSource.get();
		}

		double firstWeight = this.timeToWeights.apply(timeDelta / timeToSwitch);
		double secondWeight = 1 - firstWeight;
		if (!useFirstSource) {
			double temp = firstWeight;
			secondWeight = firstWeight;
			firstWeight = temp;
		}
		return firstSource.get() * firstWeight + secondSource.get() * secondWeight;
	}

	public void switchSources() {
		useFirstSource = !useFirstSource;
		switchingTime = TimeUtils.getCurrentTimeSeconds();
	}

	public void switchToFirstSource() {
		useFirstSource = true;
		switchingTime = TimeUtils.getCurrentTimeSeconds();
	}

	public void switchToSecondsSource() {
		useFirstSource = false;
		switchingTime = TimeUtils.getCurrentTimeSeconds();
	}

	public boolean isFirstSourceBeingUsed() {
		return useFirstSource;
	}

}
