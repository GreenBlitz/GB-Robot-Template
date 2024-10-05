package frc.robot.poseestimator.linearfilters;

import edu.wpi.first.math.filter.LinearFilter;

import java.util.LinkedList;

public class LinearFilters {

	static class HighPassIIR extends GBLinearFilter {

		private LinearFilter filter;

		public HighPassIIR(double timeConstant) {
			this.filter = LinearFilter.highPass(timeConstant, LinearFiltersConstants.MAIN_LOOP_PERIOD_TIME);
		}

		@Override
		LinearFilter getFilter() {
			return filter;
		}

	}

	static class SinglePoleIIR extends GBLinearFilter {

		private LinearFilter filter;

		public SinglePoleIIR(double timeConstant) {
			this.filter = LinearFilter.singlePoleIIR(timeConstant, LinearFiltersConstants.MAIN_LOOP_PERIOD_TIME);
		}

		@Override
		LinearFilter getFilter() {
			return filter;
		}

	}

	static class MovingAverageFIR extends GBLinearFilter {

		private LinearFilter filter;

		public MovingAverageFIR(int samples) {
			this.filter = LinearFilter.movingAverage(samples);
		}

		@Override
		LinearFilter getFilter() {
			return filter;
		}

	}

	static class GBFilter extends GBLinearFilter {

		private LinearFilter filter;

		class shittyFilter extends LinearFilter {

			private LinkedList<Double> data;
			private int sampleSize;

			public shittyFilter(int samples) {
				super(new double[] {}, new double[] {});
				sampleSize = samples;
			}

			@Override
			public double calculate(double input) {
				double output = 0;
				double weights = 0;
				data.add(input);
				if (data.size() >= sampleSize) {
					data.removeFirst();
				}
				for (int i = 0; i<=sampleSize; i++) {
					weights += 1.0 / i;
					output += data.indexOf(i) * weights;
				}

				return output / weights;
			}

			@Override
			public double lastValue() {
				return data.peek();
			}

		}

		public GBFilter(int samples) {
			this.filter = new shittyFilter(samples);
		}

		@Override
		LinearFilter getFilter() {
			return filter;
		}

	}

}
