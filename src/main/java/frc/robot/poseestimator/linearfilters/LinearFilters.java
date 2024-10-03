package frc.robot.poseestimator.linearfilters;

import edu.wpi.first.math.filter.LinearFilter;

public class LinearFilters {

    static class HighPassIIR extends ILinearFilter {

        private LinearFilter filter;

        public HighPassIIR(double timeConstant) {
            this.filter = LinearFilter.highPass(timeConstant, LinearFiltersConstants.MAIN_LOOP_PERIOD_TIME);
        }

        @Override
        LinearFilter getFilter() {
            return filter;
        }

    }

    static class SinglePoleIIR extends ILinearFilter {

        private LinearFilter filter;

        public SinglePoleIIR(double timeConstant) {
            this.filter = LinearFilter.singlePoleIIR(timeConstant, LinearFiltersConstants.MAIN_LOOP_PERIOD_TIME);
        }

        @Override
        LinearFilter getFilter() {
            return filter;
        }

    }

    static class MovingAverageFIR extends ILinearFilter {

        private LinearFilter filter;

        public MovingAverageFIR(int samples) {
            this.filter = LinearFilter.movingAverage(samples);
        }

        @Override
        LinearFilter getFilter() {
            return filter;
        }

    }

}
